#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_common_interfaces/msg/dynamic_obstacle_track_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "robot_nav/dynamic_obstacles/common.hpp"

namespace robot_nav::dynamic_obstacles
{

class DynamicObstacleCostmapBridge : public rclcpp::Node
{
public:
  DynamicObstacleCostmapBridge()
  : Node("dynamic_obstacle_costmap_bridge")
  {
    declare_parameter("input_topic", "/dynamic_obstacles/tracks");
    declare_parameter("occupancy_topic", "/dynamic_obstacles/predicted_occupancy");
    declare_parameter("marker_topic", "/dynamic_obstacles/markers");
    declare_parameter("watchdog_timeout_sec", 0.8);
    declare_parameter("occupancy_z", 0.45);
    declare_parameter("occupancy_resolution", 0.18);
    declare_parameter("footprint_padding", 0.12);
    declare_parameter("uncertainty_scale", 0.25);
    declare_parameter("max_prediction_padding", 0.35);
    declare_parameter("min_footprint_x", 0.35);
    declare_parameter("min_footprint_y", 0.35);
    declare_parameter("min_track_confidence", 0.5);
    declare_parameter("max_points", 5000);
    declare_parameter("publish_empty_on_timeout", true);

    input_topic_ = get_parameter("input_topic").as_string();
    occupancy_topic_ = get_parameter("occupancy_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    watchdog_timeout_sec_ = get_parameter("watchdog_timeout_sec").as_double();
    occupancy_z_ = get_parameter("occupancy_z").as_double();
    occupancy_resolution_ = get_parameter("occupancy_resolution").as_double();
    footprint_padding_ = get_parameter("footprint_padding").as_double();
    uncertainty_scale_ = get_parameter("uncertainty_scale").as_double();
    max_prediction_padding_ = get_parameter("max_prediction_padding").as_double();
    min_footprint_x_ = get_parameter("min_footprint_x").as_double();
    min_footprint_y_ = get_parameter("min_footprint_y").as_double();
    min_track_confidence_ = get_parameter("min_track_confidence").as_double();
    max_points_ = get_parameter("max_points").as_int();
    publish_empty_on_timeout_ = get_parameter("publish_empty_on_timeout").as_bool();

    occupancy_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(occupancy_topic_, 10);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    subscription_ = create_subscription<robot_common_interfaces::msg::DynamicObstacleTrackArray>(
      input_topic_,
      10,
      std::bind(&DynamicObstacleCostmapBridge::tracks_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&DynamicObstacleCostmapBridge::watchdog_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "dynamic_obstacle_costmap_bridge: %s -> %s, %s",
      input_topic_.c_str(),
      occupancy_topic_.c_str(),
      marker_topic_.c_str());
  }

private:
  double prediction_padding(
    const robot_common_interfaces::msg::DynamicObstacleTrack & track,
    std::size_t prediction_index) const
  {
    double uncertainty = 0.0;
    if (prediction_index < track.predicted_position_std.size()) {
      uncertainty = uncertainty_scale_ *
        static_cast<double>(track.predicted_position_std[prediction_index]);
    }
    return footprint_padding_ + std::min(max_prediction_padding_, uncertainty);
  }

  std::vector<Point3> footprint_points(
    const geometry_msgs::msg::Pose & pose,
    const robot_common_interfaces::msg::DynamicObstacleTrack & track,
    double padding) const
  {
    return oriented_footprint_points(
      pose.position.x,
      pose.position.y,
      yaw_from_quaternion(pose.orientation),
      std::max(min_footprint_x_, static_cast<double>(track.size.x)),
      std::max(min_footprint_y_, static_cast<double>(track.size.y)),
      occupancy_resolution_,
      occupancy_z_,
      padding);
  }

  std::vector<Point3> occupancy_points(
    const robot_common_interfaces::msg::DynamicObstacleTrackArray & msg) const
  {
    std::vector<Point3> points;
    if (max_points_ > 0) {
      points.reserve(static_cast<std::size_t>(max_points_));
    }
    for (const auto & track : msg.tracks) {
      if (track.confidence < min_track_confidence_) {
        continue;
      }

      auto current = footprint_points(track.pose, track, footprint_padding_);
      points.insert(points.end(), current.begin(), current.end());
      if (max_points_ > 0 && static_cast<int>(points.size()) >= max_points_) {
        points.resize(static_cast<std::size_t>(max_points_));
        return points;
      }

      for (std::size_t index = 0; index < track.predicted_poses.size(); ++index) {
        auto predicted = footprint_points(
          track.predicted_poses[index],
          track,
          prediction_padding(track, index));
        points.insert(points.end(), predicted.begin(), predicted.end());
        if (max_points_ > 0 && static_cast<int>(points.size()) >= max_points_) {
          points.resize(static_cast<std::size_t>(max_points_));
          return points;
        }
      }
    }
    return points;
  }

  visualization_msgs::msg::MarkerArray markers(
    const robot_common_interfaces::msg::DynamicObstacleTrackArray & msg) const
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header = msg.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 1;
    for (const auto & track : msg.tracks) {
      if (track.confidence < min_track_confidence_) {
        continue;
      }

      visualization_msgs::msg::Marker cube;
      cube.header = msg.header;
      cube.ns = "dynamic_obstacle_tracks";
      cube.id = marker_id++;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose = track.pose;
      cube.pose.position.z = std::max(0.25, 0.5 * static_cast<double>(track.size.z));
      cube.scale.x = std::max(0.15, static_cast<double>(track.size.x));
      cube.scale.y = std::max(0.15, static_cast<double>(track.size.y));
      cube.scale.z = std::max(0.25, static_cast<double>(track.size.z));
      cube.color.r = 1.0F;
      cube.color.g = 0.28F;
      cube.color.b = 0.08F;
      cube.color.a = 0.45F;
      cube.lifetime = rclcpp::Duration::from_seconds(1.0);
      markers.markers.push_back(cube);

      visualization_msgs::msg::Marker line;
      line.header = msg.header;
      line.ns = "dynamic_obstacle_predictions";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.07;
      line.color.r = 0.0F;
      line.color.g = 0.65F;
      line.color.b = 1.0F;
      line.color.a = 0.95F;
      line.lifetime = rclcpp::Duration::from_seconds(1.0);
      line.points.push_back(track.pose.position);
      for (const auto & pose : track.predicted_poses) {
        line.points.push_back(pose.position);
      }
      markers.markers.push_back(line);

      visualization_msgs::msg::Marker text;
      text.header = msg.header;
      text.ns = "dynamic_obstacle_labels";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = track.pose;
      text.pose.position.z = std::max(0.8, static_cast<double>(track.size.z) + 0.3);
      text.scale.z = 0.28;
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      text.color.a = 0.95F;
      text.text = "id " + std::to_string(track.id) + "  " +
        std::to_string(static_cast<double>(track.speed)).substr(0, 4) + " m/s";
      text.lifetime = rclcpp::Duration::from_seconds(1.0);
      markers.markers.push_back(text);
    }
    return markers;
  }

  void tracks_callback(
    const robot_common_interfaces::msg::DynamicObstacleTrackArray::SharedPtr msg)
  {
    last_stamp_ = stamp_seconds(msg->header.stamp);
    last_header_ = msg->header;
    timed_out_ = false;

    occupancy_publisher_->publish(cloud_from_points(msg->header, occupancy_points(*msg)));
    marker_publisher_->publish(markers(*msg));
  }

  void publish_clear()
  {
    if (!last_header_.has_value()) {
      return;
    }

    auto header = *last_header_;
    const auto now_ns = get_clock()->now().nanoseconds();
    header.stamp.sec = static_cast<std::int32_t>(now_ns / 1000000000LL);
    header.stamp.nanosec = static_cast<std::uint32_t>(now_ns % 1000000000LL);
    if (publish_empty_on_timeout_) {
      occupancy_publisher_->publish(cloud_from_points(header, {}));
    }

    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(clear);
    marker_publisher_->publish(markers);
  }

  void watchdog_callback()
  {
    if (!last_stamp_.has_value() || timed_out_) {
      return;
    }

    const double now = get_clock()->now().seconds();
    if (now - *last_stamp_ > watchdog_timeout_sec_) {
      timed_out_ = true;
      publish_clear();
    }
  }

  std::string input_topic_;
  std::string occupancy_topic_;
  std::string marker_topic_;
  double watchdog_timeout_sec_{0.0};
  double occupancy_z_{0.0};
  double occupancy_resolution_{0.0};
  double footprint_padding_{0.0};
  double uncertainty_scale_{0.0};
  double max_prediction_padding_{0.0};
  double min_footprint_x_{0.0};
  double min_footprint_y_{0.0};
  double min_track_confidence_{0.0};
  long max_points_{0};
  bool publish_empty_on_timeout_{true};

  std::optional<double> last_stamp_;
  std::optional<std_msgs::msg::Header> last_header_;
  bool timed_out_{false};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<robot_common_interfaces::msg::DynamicObstacleTrackArray>::SharedPtr
    subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_nav::dynamic_obstacles

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_nav::dynamic_obstacles::DynamicObstacleCostmapBridge>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node->get_logger(),
      "dynamic_obstacle_costmap_bridge crashed: %s",
      error.what());
  }
  rclcpp::shutdown();
  return 0;
}
