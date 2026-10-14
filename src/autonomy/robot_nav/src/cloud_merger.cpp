// cloud_merger: fuse several rigidly mounted LiDAR clouds into one PointCloud2.
//
// The perception chain (distortion_corrector, pointcloud_to_laserscan, STVL,
// collision_monitor, DLIO) consumes a single cloud on /lidar/points. Robots with
// more than one LiDAR (A2-W front + rear JT128) publish one cloud per sensor; this
// node transforms each into output_frame using the static extrinsics from TF and
// concatenates them. Output is paced by the first (primary) input: each primary
// sweep is held until every other input has delivered a sweep within
// sync_tolerance_sec of it (or has moved past it), then merged with the closest
// buffered sweep no older than max_input_age_sec. A silent secondary is not waited
// for, so a dead sensor reduces coverage instead of stalling perception; a held
// sweep is also flushed when the next primary sweep arrives.
//
// Inputs must share the same point layout (fields + point_step), which holds for
// identical sensors behind the same driver. Point time fields are copied as-is.

#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class CloudMerger : public rclcpp::Node
{
public:
  CloudMerger()
  : Node("cloud_merger")
  {
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics", {"/lidar/front/points", "/lidar/rear/points"});
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points");
    output_frame_ = declare_parameter<std::string>("output_frame", "lidar_link");
    max_input_age_sec_ = declare_parameter<double>("max_input_age_sec", 0.15);
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", 0.06);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);

    if (input_topics_.empty()) {
      throw std::runtime_error("cloud_merger: input_topics must not be empty");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // RELIABLE output: DLIO subscribes /lidar/points as reliable and would otherwise
    // receive nothing; best-effort consumers still match a reliable publisher.
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());

    history_.resize(input_topics_.size());
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(i, msg); }));
    }

    RCLCPP_INFO(
      get_logger(), "Merging %zu clouds into %s (frame %s), primary %s",
      input_topics_.size(), output_topic_.c_str(), output_frame_.c_str(),
      input_topics_[0].c_str());
  }

private:
  void on_cloud(size_t index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    if (index == 0) {
      if (pending_) {
        publish_merged(pending_);
      }
      pending_ = msg;
    } else {
      auto & history = history_[index];
      history.push_back(msg);
      while (history.size() > kHistoryDepth) {
        history.pop_front();
      }
    }
    if (pending_ && secondaries_ready(rclcpp::Time(pending_->header.stamp))) {
      publish_merged(pending_);
      pending_.reset();
    }
  }

  // A secondary is ready when it holds a sweep within sync_tolerance of the
  // primary, has already moved past the primary stamp (that sweep was skipped),
  // or has been silent for longer than max_input_age.
  bool secondaries_ready(const rclcpp::Time & primary_stamp) const
  {
    for (size_t i = 1; i < history_.size(); ++i) {
      const auto & history = history_[i];
      if (history.empty()) {
        continue;
      }
      bool matched = false;
      for (const auto & cloud : history) {
        const double dt = (rclcpp::Time(cloud->header.stamp) - primary_stamp).seconds();
        matched = matched || std::abs(dt) <= sync_tolerance_sec_;
      }
      const double newest_dt =
        (rclcpp::Time(history.back()->header.stamp) - primary_stamp).seconds();
      const bool passed = newest_dt > sync_tolerance_sec_;
      const bool silent = newest_dt < -max_input_age_sec_;
      if (!matched && !passed && !silent) {
        return false;
      }
    }
    return true;
  }

  sensor_msgs::msg::PointCloud2::ConstSharedPtr closest_sweep(
    size_t index, const rclcpp::Time & primary_stamp) const
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr best;
    double best_dt = max_input_age_sec_;
    for (const auto & cloud : history_[index]) {
      const double dt = std::abs((rclcpp::Time(cloud->header.stamp) - primary_stamp).seconds());
      if (dt <= best_dt) {
        best_dt = dt;
        best = cloud;
      }
    }
    return best;
  }

  bool to_output_frame(
    const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out)
  {
    if (in.header.frame_id == output_frame_) {
      out = in;
      return true;
    }
    try {
      // Sensors are rigidly mounted: the latest static extrinsic is exact.
      const auto tf = tf_buffer_->lookupTransform(
        output_frame_, in.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      tf2::doTransform(in, out, tf);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No TF %s -> %s: %s",
        in.header.frame_id.c_str(), output_frame_.c_str(), ex.what());
      return false;
    }
  }

  static bool same_layout(
    const sensor_msgs::msg::PointCloud2 & a, const sensor_msgs::msg::PointCloud2 & b)
  {
    if (a.point_step != b.point_step || a.fields.size() != b.fields.size() ||
      a.is_bigendian != b.is_bigendian)
    {
      return false;
    }
    for (size_t i = 0; i < a.fields.size(); ++i) {
      if (a.fields[i].name != b.fields[i].name || a.fields[i].offset != b.fields[i].offset ||
        a.fields[i].datatype != b.fields[i].datatype)
      {
        return false;
      }
    }
    return true;
  }

  void publish_merged(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & primary)
  {
    auto merged = std::make_unique<sensor_msgs::msg::PointCloud2>();
    if (!to_output_frame(*primary, *merged)) {
      return;
    }
    merged->header.stamp = primary->header.stamp;
    merged->header.frame_id = output_frame_;
    // Flatten: merged data is no longer an organized ring/column grid.
    merged->width = merged->width * merged->height;
    merged->height = 1;
    merged->row_step = merged->width * merged->point_step;
    merged->is_dense = false;

    const rclcpp::Time primary_stamp(primary->header.stamp);
    for (size_t i = 1; i < history_.size(); ++i) {
      const auto cloud = closest_sweep(i, primary_stamp);
      if (!cloud) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "No sweep from %s within %.3f s of primary",
          input_topics_[i].c_str(), max_input_age_sec_);
        continue;
      }
      sensor_msgs::msg::PointCloud2 transformed;
      if (!to_output_frame(*cloud, transformed)) {
        continue;
      }
      if (!same_layout(*merged, transformed)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Skipping %s: point layout differs from primary",
          input_topics_[i].c_str());
        continue;
      }
      merged->data.insert(merged->data.end(), transformed.data.begin(), transformed.data.end());
      merged->width += transformed.width * transformed.height;
    }
    merged->row_step = merged->width * merged->point_step;
    pub_->publish(std::move(merged));
  }

  std::vector<std::string> input_topics_;
  std::string output_topic_;
  std::string output_frame_;
  double max_input_age_sec_;
  double sync_tolerance_sec_;
  double tf_timeout_sec_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  static constexpr size_t kHistoryDepth = 5;
  std::vector<std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr>> history_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudMerger>());
  rclcpp::shutdown();
  return 0;
}
