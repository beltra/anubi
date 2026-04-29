#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

#include "robot_nav/dynamic_obstacles/common.hpp"

namespace robot_nav::dynamic_obstacles
{

namespace
{

struct VoxelKey
{
  long x{0};
  long y{0};
  long z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<long>{}(key.x);
    const auto hy = std::hash<long>{}(key.y);
    const auto hz = std::hash<long>{}(key.z);
    return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2)) ^
           (hz + 0x9e3779b9 + (hy << 6) + (hy >> 2));
  }
};

std::vector<Point3> preprocess_points(
  const sensor_msgs::msg::PointCloud2 & msg,
  const Eigen::Matrix4d & transform,
  double min_z,
  double max_z,
  double voxel_size,
  int max_points)
{
  const auto input_points = points_from_cloud(msg);
  if (input_points.empty()) {
    return {};
  }

  std::vector<Point3> output;
  output.reserve(input_points.size());

  std::unordered_set<VoxelKey, VoxelKeyHash> seen_voxels;
  const bool use_voxels = voxel_size > 0.0;
  if (use_voxels) {
    seen_voxels.reserve(input_points.size());
  }

  for (const auto & point : input_points) {
    const Eigen::Vector4d hom(point.x(), point.y(), point.z(), 1.0);
    const Eigen::Vector4d transformed = transform * hom;
    const Point3 output_point(transformed.x(), transformed.y(), transformed.z());
    if (output_point.z() < min_z || output_point.z() > max_z) {
      continue;
    }

    if (use_voxels) {
      const VoxelKey key{
        static_cast<long>(std::floor(output_point.x() / voxel_size)),
        static_cast<long>(std::floor(output_point.y() / voxel_size)),
        static_cast<long>(std::floor(output_point.z() / voxel_size)),
      };
      if (!seen_voxels.insert(key).second) {
        continue;
      }
    }

    output.push_back(output_point);
  }

  if (max_points > 0 && static_cast<int>(output.size()) > max_points) {
    const std::size_t step = std::max<std::size_t>(1U, output.size() / max_points);
    std::vector<Point3> limited;
    limited.reserve(static_cast<std::size_t>(max_points));
    for (
      std::size_t idx = 0;
      idx < output.size() && limited.size() < static_cast<std::size_t>(max_points);
      idx += step)
    {
      limited.push_back(output[idx]);
    }
    output = std::move(limited);
  }

  return output;
}

}  // namespace

class DynamicObstaclePreprocessor : public rclcpp::Node
{
public:
  DynamicObstaclePreprocessor()
  : Node("dynamic_obstacle_preprocessor"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("input_topic", "/lidar/points_undistorted");
    declare_parameter("output_topic", "/dynamic_obstacles/points_world");
    declare_parameter("fixed_frame", "map");
    declare_parameter("min_z", 0.08);
    declare_parameter("max_z", 2.0);
    declare_parameter("voxel_size", 0.12);
    declare_parameter("max_points", 6000);
    declare_parameter("tf_timeout_sec", 0.05);
    declare_parameter("fallback_to_latest_tf", false);
    declare_parameter("max_latest_tf_age_sec", 0.05);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    fixed_frame_ = get_parameter("fixed_frame").as_string();
    min_z_ = get_parameter("min_z").as_double();
    max_z_ = get_parameter("max_z").as_double();
    voxel_size_ = get_parameter("voxel_size").as_double();
    max_points_ = get_parameter("max_points").as_int();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    fallback_to_latest_tf_ = get_parameter("fallback_to_latest_tf").as_bool();
    max_latest_tf_age_sec_ = get_parameter("max_latest_tf_age_sec").as_double();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DynamicObstaclePreprocessor::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "dynamic_obstacle_preprocessor: %s -> %s in %s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      fixed_frame_.c_str());
  }

private:
  geometry_msgs::msg::TransformStamped lookup_transform(
    const sensor_msgs::msg::PointCloud2 & msg)
  {
    const rclcpp::Time msg_stamp(msg.header.stamp);
    try {
      return tf_buffer_.lookupTransform(
        fixed_frame_,
        msg.header.frame_id,
        msg_stamp,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & primary_error) {
      if (!fallback_to_latest_tf_) {
        throw;
      }

      try {
        const auto latest = tf_buffer_.lookupTransform(
          fixed_frame_,
          msg.header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_sec_));
        const double age = std::abs(
          stamp_seconds(msg.header.stamp) - stamp_seconds(latest.header.stamp));
        if (age <= max_latest_tf_age_sec_) {
          return latest;
        }
      } catch (const tf2::TransformException &) {
      }
      throw primary_error;
    }
  }

  void warn_throttled(const std::string & message)
  {
    const double now = get_clock()->now().seconds();
    if (now - last_warn_time_sec_ > 2.0) {
      last_warn_time_sec_ = now;
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = lookup_transform(*msg);
    } catch (const tf2::TransformException & error) {
      if (!has_valid_transform_ && startup_tf_failures_ < startup_tf_grace_failures_) {
        ++startup_tf_failures_;
        return;
      }
      warn_throttled(
        std::string("Skipping dynamic obstacle cloud: TF unavailable (") + error.what() + ")");
      return;
    }
    has_valid_transform_ = true;

    const auto points = preprocess_points(
      *msg,
      transform_to_matrix(transform),
      min_z_,
      max_z_,
      voxel_size_,
      max_points_);

    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = fixed_frame_;
    publisher_->publish(cloud_from_points(header, points));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string fixed_frame_;
  double min_z_{0.0};
  double max_z_{0.0};
  double voxel_size_{0.0};
  int max_points_{0};
  double tf_timeout_sec_{0.0};
  bool fallback_to_latest_tf_{false};
  double max_latest_tf_age_sec_{0.0};
  double last_warn_time_sec_{0.0};
  int startup_tf_failures_{0};
  const int startup_tf_grace_failures_{10};
  bool has_valid_transform_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace robot_nav::dynamic_obstacles

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_nav::dynamic_obstacles::DynamicObstaclePreprocessor>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "dynamic_obstacle_preprocessor crashed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
