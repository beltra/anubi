#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_common_interfaces/msg/dynamic_obstacle_cluster.hpp"
#include "robot_common_interfaces/msg/dynamic_obstacle_cluster_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_nav/dynamic_obstacles/common.hpp"

namespace robot_nav::dynamic_obstacles
{
namespace
{

struct PreviousCluster
{
  Point3 centroid{Point3::Zero()};
  double stamp{0.0};
  int misses{0};
  int dynamic_hits{0};
  int consecutive_dynamic_hits{0};
  int stationary_hits{1};
};

struct DynamicCandidate
{
  double sensor_range{0.0};
  Point3 centroid{Point3::Zero()};
  Point3 size{Point3::Zero()};
  Point3 velocity{Point3::Zero()};
  double yaw{0.0};
  int cluster_id{0};
  std::vector<Point3> points;
  int point_count{0};
};

}  // namespace

class DynamicClusterDetector : public rclcpp::Node
{
public:
  DynamicClusterDetector()
  : Node("dynamic_cluster_detector"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("input_topic", "/dynamic_obstacles/points_world");
    declare_parameter("output_topic", "/dynamic_obstacles/clusters");
    declare_parameter("map_topic", "/map");
    declare_parameter("use_static_map", true);
    declare_parameter("require_static_map", true);
    declare_parameter("occupied_threshold", 65);
    declare_parameter("static_inflation_radius", 0.18);
    declare_parameter("cluster_tolerance", 0.35);
    declare_parameter("min_cluster_points", 4);
    declare_parameter("max_cluster_points", 1200);
    declare_parameter("max_clusters", 24);
    declare_parameter("association_distance", 0.75);
    declare_parameter("min_dynamic_displacement", 0.12);
    declare_parameter("min_dynamic_speed", 0.15);
    declare_parameter("min_dynamic_hits", 1);
    declare_parameter("max_track_misses", 5);
    declare_parameter("min_cluster_size_xy", 0.08);
    declare_parameter("max_cluster_size_xy", 3.2);
    declare_parameter("wall_sliver_min_length", 0.75);
    declare_parameter("wall_sliver_max_length", 0.0);
    declare_parameter("wall_sliver_max_width", 0.16);
    declare_parameter("static_map_occupied_ratio_threshold", 0.5);
    declare_parameter("static_map_large_cluster_min_length", 1.4);
    declare_parameter("static_map_large_cluster_max_length", 2.2);
    declare_parameter("static_map_large_cluster_max_width", 1.2);
    declare_parameter("static_shape_min_length", 1.4);
    declare_parameter("static_shape_max_length", 2.2);
    declare_parameter("static_shape_max_width", 1.2);
    declare_parameter("shadow_management_enabled", true);
    declare_parameter("sensor_frame", "lidar_stabilized");
    declare_parameter("shadow_tf_timeout_sec", 0.03);
    declare_parameter("shadow_persistence_sec", 0.75);
    declare_parameter("shadow_angle_margin_rad", 0.06);
    declare_parameter("shadow_range_padding", 0.15);
    declare_parameter("shadow_max_range", 12.0);
    declare_parameter("odom_topic", "/odom");
    declare_parameter("ego_velocity_rejection", true);
    declare_parameter("ego_speed_ratio_threshold", 0.55);
    declare_parameter("ego_min_robot_speed", 0.08);
    declare_parameter("ego_artifact_min_speed_ratio", 0.35);
    declare_parameter("ego_artifact_max_speed_ratio", 1.8);
    declare_parameter("ego_artifact_direction_cosine", 0.75);
    declare_parameter("filter_out_of_map", true);
    declare_parameter("min_consecutive_dynamic_hits", 1);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    use_static_map_ = get_parameter("use_static_map").as_bool();
    require_static_map_ = get_parameter("require_static_map").as_bool();
    occupied_threshold_ = get_parameter("occupied_threshold").as_int();
    static_inflation_radius_ = get_parameter("static_inflation_radius").as_double();
    cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
    min_cluster_points_ = get_parameter("min_cluster_points").as_int();
    max_cluster_points_ = get_parameter("max_cluster_points").as_int();
    max_clusters_ = get_parameter("max_clusters").as_int();
    association_distance_ = get_parameter("association_distance").as_double();
    min_dynamic_displacement_ = get_parameter("min_dynamic_displacement").as_double();
    min_dynamic_speed_ = get_parameter("min_dynamic_speed").as_double();
    min_dynamic_hits_ = std::max(1L, get_parameter("min_dynamic_hits").as_int());
    max_track_misses_ = get_parameter("max_track_misses").as_int();
    min_cluster_size_xy_ = get_parameter("min_cluster_size_xy").as_double();
    max_cluster_size_xy_ = get_parameter("max_cluster_size_xy").as_double();
    wall_sliver_min_length_ = get_parameter("wall_sliver_min_length").as_double();
    wall_sliver_max_length_ = get_parameter("wall_sliver_max_length").as_double();
    wall_sliver_max_width_ = get_parameter("wall_sliver_max_width").as_double();
    static_map_occupied_ratio_threshold_ =
      get_parameter("static_map_occupied_ratio_threshold").as_double();
    static_map_large_cluster_min_length_ =
      get_parameter("static_map_large_cluster_min_length").as_double();
    static_map_large_cluster_max_length_ =
      get_parameter("static_map_large_cluster_max_length").as_double();
    static_map_large_cluster_max_width_ =
      get_parameter("static_map_large_cluster_max_width").as_double();
    static_shape_min_length_ = get_parameter("static_shape_min_length").as_double();
    static_shape_max_length_ = get_parameter("static_shape_max_length").as_double();
    static_shape_max_width_ = get_parameter("static_shape_max_width").as_double();
    shadow_management_enabled_ = get_parameter("shadow_management_enabled").as_bool();
    sensor_frame_ = get_parameter("sensor_frame").as_string();
    shadow_tf_timeout_sec_ = get_parameter("shadow_tf_timeout_sec").as_double();
    shadow_persistence_sec_ = get_parameter("shadow_persistence_sec").as_double();
    shadow_angle_margin_rad_ = get_parameter("shadow_angle_margin_rad").as_double();
    shadow_range_padding_ = get_parameter("shadow_range_padding").as_double();
    shadow_max_range_ = get_parameter("shadow_max_range").as_double();
    odom_topic_ = get_parameter("odom_topic").as_string();
    ego_velocity_rejection_ = get_parameter("ego_velocity_rejection").as_bool();
    ego_speed_ratio_threshold_ = get_parameter("ego_speed_ratio_threshold").as_double();
    ego_min_robot_speed_ = get_parameter("ego_min_robot_speed").as_double();
    ego_artifact_min_speed_ratio_ = get_parameter("ego_artifact_min_speed_ratio").as_double();
    ego_artifact_max_speed_ratio_ = get_parameter("ego_artifact_max_speed_ratio").as_double();
    ego_artifact_direction_cosine_ = get_parameter("ego_artifact_direction_cosine").as_double();
    filter_out_of_map_ = get_parameter("filter_out_of_map").as_bool();
    min_consecutive_dynamic_hits_ = std::max(
      1L, get_parameter("min_consecutive_dynamic_hits").as_int());

    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();

    publisher_ =
      create_publisher<robot_common_interfaces::msg::DynamicObstacleClusterArray>(output_topic_, 10);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DynamicClusterDetector::cloud_callback, this, std::placeholders::_1));
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      map_qos,
      std::bind(&DynamicClusterDetector::map_callback, this, std::placeholders::_1));
    if (ego_velocity_rejection_) {
      odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&DynamicClusterDetector::odom_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "dynamic_cluster_detector: %s -> %s (static_map=%s, require_map=%s, "
      "shadow_management=%s, ego_rejection=%s, out_of_map_filter=%s)",
      input_topic_.c_str(),
      output_topic_.c_str(),
      use_static_map_ ? "true" : "false",
      require_static_map_ ? "true" : "false",
      shadow_management_enabled_ ? "true" : "false",
      ego_velocity_rejection_ ? "true" : "false",
      filter_out_of_map_ ? "true" : "false");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ego_vx_ = msg->twist.twist.linear.x;
    ego_vy_ = msg->twist.twist.linear.y;
    ego_speed_ = std::hypot(ego_vx_, ego_vy_);
    ego_yaw_rate_ = std::abs(msg->twist.twist.angular.z);
  }

  bool is_static_point(double x, double y) const
  {
    if (!use_static_map_ || !map_) {
      return false;
    }

    const auto & info = map_->info;
    const double res = info.resolution;
    if (res <= 0.0) {
      return false;
    }

    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;
    const int mx = static_cast<int>(std::floor((x - ox) / res));
    const int my = static_cast<int>(std::floor((y - oy) / res));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) {
      return filter_out_of_map_;
    }

    const int radius_cells = std::max(0, static_cast<int>(std::ceil(static_inflation_radius_ / res)));
    for (int yy = std::max(0, my - radius_cells);
      yy < std::min(static_cast<int>(info.height), my + radius_cells + 1); ++yy)
    {
      const int row = yy * static_cast<int>(info.width);
      for (int xx = std::max(0, mx - radius_cells);
        xx < std::min(static_cast<int>(info.width), mx + radius_cells + 1); ++xx)
      {
        if (map_->data[static_cast<std::size_t>(row + xx)] >= occupied_threshold_) {
          return true;
        }
      }
    }
    return false;
  }

  std::vector<Point3> foreground_points(const std::vector<Point3> & points) const
  {
    if (points.empty()) {
      return {};
    }
    if (!use_static_map_ || !map_) {
      return points;
    }

    std::vector<Point3> out;
    out.reserve(points.size());
    for (const auto & point : points) {
      if (!is_static_point(point.x(), point.y())) {
        out.push_back(point);
      }
    }
    return out;
  }

  bool is_out_of_map(double x, double y) const
  {
    if (!use_static_map_ || !map_) {
      return false;
    }

    const auto & info = map_->info;
    const double res = info.resolution;
    if (res <= 0.0) {
      return false;
    }

    const int mx = static_cast<int>(std::floor((x - info.origin.position.x) / res));
    const int my = static_cast<int>(std::floor((y - info.origin.position.y) / res));
    return mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
           my >= static_cast<int>(info.height);
  }

  bool is_out_of_map_cluster(const std::vector<Point3> & points) const
  {
    if (!filter_out_of_map_ || points.empty()) {
      return false;
    }

    int out_of_map = 0;
    for (const auto & point : points) {
      out_of_map += is_out_of_map(point.x(), point.y()) ? 1 : 0;
    }
    return static_cast<double>(out_of_map) / static_cast<double>(points.size()) >= 0.5;
  }

  bool is_static_background_cluster(const std::vector<Point3> & points, const Point3 & size) const
  {
    if (!use_static_map_ || !map_ || points.empty()) {
      return false;
    }

    int occupied = 0;
    for (const auto & point : points) {
      occupied += is_static_point(point.x(), point.y()) ? 1 : 0;
    }
    const double occupied_ratio =
      static_cast<double>(occupied) / static_cast<double>(points.size());
    if (occupied_ratio < static_map_occupied_ratio_threshold_) {
      return false;
    }

    const double long_axis = std::max(size.x(), size.y());
    const double short_axis = std::min(size.x(), size.y());
    return long_axis >= static_map_large_cluster_min_length_ &&
           (static_map_large_cluster_max_length_ <= 0.0 ||
           long_axis <= static_map_large_cluster_max_length_) &&
           short_axis <= static_map_large_cluster_max_width_;
  }

  std::optional<Point2> sensor_xy(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!shadow_management_enabled_) {
      return std::nullopt;
    }

    const auto warn_missing_shadow_tf =
      [this, &msg](const std::string & reason)
      {
        if (!has_shadow_tf_ && startup_shadow_tf_failures_ < startup_shadow_tf_grace_failures_) {
          ++startup_shadow_tf_failures_;
          return;
        }
        const double now = get_clock()->now().seconds();
        if (now - last_shadow_tf_warn_time_sec_ < 2.0) {
          return;
        }
        last_shadow_tf_warn_time_sec_ = now;
        RCLCPP_WARN(
          get_logger(),
          "Shadow-cone TF unavailable for %s <- %s at %.3f s (%s); occlusion filtering is disabled for this frame",
          msg.header.frame_id.c_str(),
          sensor_frame_.c_str(),
          stamp_seconds(msg.header.stamp),
          reason.c_str());
      };

    try {
      const auto transform = tf_buffer_.lookupTransform(
        msg.header.frame_id,
        sensor_frame_,
        rclcpp::Time(msg.header.stamp),
        tf2::durationFromSec(shadow_tf_timeout_sec_));
      has_shadow_tf_ = true;
      return Point2(transform.transform.translation.x, transform.transform.translation.y);
    } catch (const tf2::TransformException & primary_error) {
      try {
        const auto latest = tf_buffer_.lookupTransform(
          msg.header.frame_id,
          sensor_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(shadow_tf_timeout_sec_));
        const double age =
          std::abs(stamp_seconds(msg.header.stamp) - stamp_seconds(latest.header.stamp));
        if (age <= 0.05) {
          has_shadow_tf_ = true;
          return Point2(latest.transform.translation.x, latest.transform.translation.y);
        }
        warn_missing_shadow_tf(
          "latest transform too old after exact-time lookup failed: " + std::to_string(age) +
          " s, primary error: " + primary_error.what());
      } catch (const tf2::TransformException & latest_error) {
        warn_missing_shadow_tf(
          std::string("exact-time lookup failed: ") + primary_error.what() +
          "; latest lookup failed: " + latest_error.what());
      }
      return std::nullopt;
    }
  }

  void expire_shadow_cones(double stamp)
  {
    shadow_cones_.erase(
      std::remove_if(
        shadow_cones_.begin(),
        shadow_cones_.end(),
        [stamp](const ShadowCone & cone) {return cone.expires_at < stamp;}),
      shadow_cones_.end());
  }

  void update_sensor_motion(const std::optional<Point2> & sensor_xy, double stamp)
  {
    if (!sensor_xy.has_value()) {
      return;
    }

    if (last_sensor_xy_.has_value() && last_sensor_stamp_.has_value()) {
      const double dt = stamp - *last_sensor_stamp_;
      if (dt >= 1.0e-3 && dt <= 0.5) {
        const Point2 measured = (*sensor_xy - *last_sensor_xy_) / dt;
        ego_map_velocity_ = 0.6 * ego_map_velocity_ + 0.4 * measured;
        ego_map_speed_ = ego_map_velocity_.norm();
      }
    }

    last_sensor_xy_ = sensor_xy;
    last_sensor_stamp_ = stamp;
  }

  std::vector<Point3> remove_shadowed_points(const std::vector<Point3> & points) const
  {
    if (!shadow_management_enabled_ || shadow_cones_.empty() || points.empty()) {
      return points;
    }

    const auto mask = points_in_shadow_cones(points, shadow_cones_);
    std::vector<Point3> out;
    out.reserve(points.size());
    for (std::size_t idx = 0; idx < points.size(); ++idx) {
      if (!mask[idx]) {
        out.push_back(points[idx]);
      }
    }
    return out;
  }

  bool is_shadowed_cluster(
    const Point3 & centroid,
    const std::vector<Point3> & points,
    const std::vector<ShadowCone> & cones) const
  {
    if (!shadow_management_enabled_ || cones.empty()) {
      return false;
    }

    if (points_in_shadow_cones({centroid}, cones)[0]) {
      return true;
    }
    if (points.empty()) {
      return false;
    }

    const auto mask = points_in_shadow_cones(points, cones);
    int shadowed = 0;
    for (bool value : mask) {
      shadowed += value ? 1 : 0;
    }
    return static_cast<double>(shadowed) / static_cast<double>(points.size()) >= 0.5;
  }

  std::optional<ShadowCone> make_shadow_cone(
    const std::optional<Point2> & sensor_xy,
    const Point3 & centroid,
    const Point3 & size,
    double stamp) const
  {
    if (!shadow_management_enabled_ || !sensor_xy.has_value()) {
      return std::nullopt;
    }

    bool valid = false;
    const auto cone = shadow_cone_from_obstacle(
      *sensor_xy,
      centroid.head<2>(),
      size.head<2>(),
      stamp,
      shadow_angle_margin_rad_,
      shadow_range_padding_,
      shadow_max_range_,
      shadow_persistence_sec_,
      &valid);
    if (!valid) {
      return std::nullopt;
    }
    return cone;
  }

  std::map<int, int> associate(const std::vector<Point3> & centroids) const
  {
    std::map<int, int> assignments;
    if (centroids.empty()) {
      return assignments;
    }

    std::vector<int> available;
    available.reserve(previous_.size());
    for (const auto & item : previous_) {
      available.push_back(item.first);
    }

    for (std::size_t idx = 0; idx < centroids.size(); ++idx) {
      int best_id = -1;
      double best_dist = association_distance_;
      for (const auto prev_id : available) {
        const double dist =
          (centroids[idx].head<2>() - previous_.at(prev_id).centroid.head<2>()).norm();
        if (dist < best_dist) {
          best_dist = dist;
          best_id = prev_id;
        }
      }

      if (best_id >= 0) {
        assignments[static_cast<int>(idx)] = best_id;
        available.erase(std::remove(available.begin(), available.end(), best_id), available.end());
      }
    }
    return assignments;
  }

  std::tuple<Point3, Point3, double> cluster_shape(const std::vector<Point3> & cluster_points) const
  {
    Point3 centroid = Point3::Zero();
    Point3 mins = Point3(
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max());
    Point3 maxs = Point3(
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest());

    for (const auto & point : cluster_points) {
      centroid += point;
      mins = mins.cwiseMin(point);
      maxs = maxs.cwiseMax(point);
    }
    centroid /= static_cast<double>(cluster_points.size());

    Point3 size = (maxs - mins).cwiseMax(Point3(min_cluster_size_xy_, min_cluster_size_xy_, 0.05));

    double yaw = 0.0;
    if (cluster_points.size() >= 3) {
      Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
      for (const auto & point : cluster_points) {
        const Eigen::Vector2d centered = point.head<2>() - centroid.head<2>();
        cov += centered * centered.transpose();
      }
      cov /= std::max<std::size_t>(1U, cluster_points.size() - 1U);

      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
      const Eigen::Vector2d axis = solver.eigenvectors().col(1);
      yaw = std::atan2(axis.y(), axis.x());
    }

    return {centroid, size, yaw};
  }

  bool is_plausible_dynamic_shape(const Point3 & size) const
  {
    const double long_axis = std::max(size.x(), size.y());
    const double short_axis = std::min(size.x(), size.y());
    if (max_cluster_size_xy_ > 0.0 && long_axis > max_cluster_size_xy_) {
      return false;
    }
    if (
      long_axis >= wall_sliver_min_length_ &&
      (wall_sliver_max_length_ <= 0.0 || long_axis <= wall_sliver_max_length_) &&
      short_axis <= wall_sliver_max_width_)
    {
      return false;
    }
    if (
      long_axis >= static_shape_min_length_ &&
      (static_shape_max_length_ <= 0.0 || long_axis <= static_shape_max_length_) &&
      short_axis <= static_shape_max_width_)
    {
      return false;
    }
    return true;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    robot_common_interfaces::msg::DynamicObstacleClusterArray out;
    out.header = msg->header;

    if (use_static_map_ && require_static_map_ && !map_) {
      publisher_->publish(out);
      return;
    }

    const double stamp = stamp_seconds(msg->header.stamp);
    expire_shadow_cones(stamp);
    const auto sensor_xy_value = sensor_xy(*msg);
    update_sensor_motion(sensor_xy_value, stamp);

    auto points = remove_shadowed_points(points_from_cloud(*msg));
    const auto clusters = euclidean_clusters(
      points,
      cluster_tolerance_,
      static_cast<int>(min_cluster_points_),
      static_cast<int>(max_cluster_points_),
      static_cast<int>(max_clusters_));

    std::vector<std::vector<std::size_t>> filtered_clusters;
    filtered_clusters.reserve(clusters.size());
    std::vector<std::tuple<Point3, Point3, double>> shapes;
    shapes.reserve(clusters.size());
    std::vector<Point3> centroids;
    centroids.reserve(clusters.size());
    for (const auto & cluster_indices : clusters) {
      std::vector<Point3> cluster_points;
      cluster_points.reserve(cluster_indices.size());
      for (const auto index : cluster_indices) {
        cluster_points.push_back(points[index]);
      }
      auto shape = cluster_shape(cluster_points);
      if (!is_plausible_dynamic_shape(std::get<1>(shape))) {
        continue;
      }
      if (is_out_of_map_cluster(cluster_points)) {
        continue;
      }
      if (is_static_background_cluster(cluster_points, std::get<1>(shape))) {
        continue;
      }
      filtered_clusters.push_back(cluster_indices);
      centroids.push_back(std::get<0>(shape));
      shapes.push_back(std::move(shape));
    }

    const auto assignments = associate(centroids);
    std::vector<int> seen_ids;
    seen_ids.reserve(filtered_clusters.size());
    std::vector<DynamicCandidate> dynamic_candidates;

    for (std::size_t idx = 0; idx < shapes.size(); ++idx) {
      const auto & centroid = std::get<0>(shapes[idx]);
      Point3 size = std::get<1>(shapes[idx]);
      double yaw = std::get<2>(shapes[idx]);
      int cluster_id = assignments.count(static_cast<int>(idx)) > 0 ?
        assignments.at(static_cast<int>(idx)) : next_id_++;

      Point3 velocity = Point3::Zero();
      int dynamic_hits = 0;
      int consecutive_dynamic_hits = 0;
      int stationary_hits = 1;
      bool dynamic = false;

      const auto prev_it = previous_.find(cluster_id);
      if (prev_it != previous_.end()) {
        const auto & prev = prev_it->second;
        stationary_hits = prev.stationary_hits;
        const double dt = std::max(1.0e-3, stamp - prev.stamp);
        const Point3 displacement = centroid - prev.centroid;
        velocity = displacement / dt;
        const double speed_xy = velocity.head<2>().norm();
        bool instant_dynamic =
          displacement.head<2>().norm() >= min_dynamic_displacement_ &&
          speed_xy >= min_dynamic_speed_;

        if (instant_dynamic && ego_velocity_rejection_) {
          const double effective_ego_speed = std::max(ego_speed_, ego_map_speed_);
          const bool ego_moving_linear = effective_ego_speed >= ego_min_robot_speed_;
          const bool ego_rotating = ego_yaw_rate_ >= 0.05;

          const bool ego_correlated = velocity_matches_ego_motion(
            velocity.head<2>(),
            ego_map_velocity_,
            ego_min_robot_speed_,
            ego_artifact_min_speed_ratio_,
            ego_artifact_max_speed_ratio_,
            ego_artifact_direction_cosine_);
          if (ego_correlated && (prev.stationary_hits >= 2 || speed_xy <= effective_ego_speed * 1.2)) {
            instant_dynamic = false;
          }

          if (ego_moving_linear && instant_dynamic) {
            if (speed_xy < effective_ego_speed * ego_speed_ratio_threshold_) {
              instant_dynamic = false;
            }
          }

          if (ego_rotating && instant_dynamic) {
            double obs_range = centroid.head<2>().norm();
            if (sensor_xy_value.has_value()) {
              obs_range = (centroid.head<2>() - *sensor_xy_value).norm();
            }
            const double rotation_induced_speed =
              ego_yaw_rate_ * obs_range * ego_speed_ratio_threshold_;
            if (speed_xy < rotation_induced_speed) {
              instant_dynamic = false;
            }
          }
        }

        if (instant_dynamic) {
          dynamic_hits = prev.dynamic_hits + 1;
          consecutive_dynamic_hits = prev.consecutive_dynamic_hits + 1;
          stationary_hits = std::max(0, prev.stationary_hits - 1);
        } else {
          dynamic_hits = std::max(0, prev.dynamic_hits - 1);
          consecutive_dynamic_hits = 0;
          stationary_hits = std::min(prev.stationary_hits + 1, 1000);
        }

        dynamic =
          dynamic_hits >= min_dynamic_hits_ &&
          consecutive_dynamic_hits >= min_consecutive_dynamic_hits_;
        if (dynamic) {
          yaw = std::atan2(velocity.y(), velocity.x());
        }
      }

      seen_ids.push_back(cluster_id);
      previous_[cluster_id] = PreviousCluster{
        centroid,
        stamp,
        0,
        dynamic_hits,
        consecutive_dynamic_hits,
        stationary_hits,
      };

      if (dynamic) {
        DynamicCandidate candidate;
        candidate.centroid = centroid;
        candidate.size = size;
        candidate.velocity = velocity;
        candidate.yaw = yaw;
        candidate.cluster_id = cluster_id;
        candidate.point_count = static_cast<int>(filtered_clusters[idx].size());
        candidate.points.reserve(filtered_clusters[idx].size());
        for (const auto point_index : filtered_clusters[idx]) {
          candidate.points.push_back(points[point_index]);
        }
        candidate.sensor_range = sensor_xy_value.has_value() ?
          (centroid.head<2>() - *sensor_xy_value).norm() : 0.0;
        dynamic_candidates.push_back(std::move(candidate));
      }
    }

    for (auto it = previous_.begin(); it != previous_.end();) {
      if (std::find(seen_ids.begin(), seen_ids.end(), it->first) == seen_ids.end()) {
        it->second.misses += 1;
        if (it->second.misses > max_track_misses_) {
          it = previous_.erase(it);
          continue;
        }
      }
      ++it;
    }

    std::sort(
      dynamic_candidates.begin(),
      dynamic_candidates.end(),
      [](const DynamicCandidate & lhs, const DynamicCandidate & rhs) {
        return lhs.sensor_range < rhs.sensor_range;
      });

    std::vector<ShadowCone> frame_cones;
    for (auto & candidate : dynamic_candidates) {
      if (is_shadowed_cluster(candidate.centroid, candidate.points, frame_cones)) {
        continue;
      }

      if (const auto cone = make_shadow_cone(sensor_xy_value, candidate.centroid, candidate.size, stamp)) {
        frame_cones.push_back(*cone);
        shadow_cones_.push_back(*cone);
      }

      robot_common_interfaces::msg::DynamicObstacleCluster cluster;
      cluster.id = static_cast<std::uint32_t>(candidate.cluster_id);
      cluster.centroid = point_msg(candidate.centroid);
      cluster.velocity = vector_msg(candidate.velocity);
      cluster.size = vector_msg(candidate.size);
      cluster.yaw = static_cast<float>(candidate.yaw);
      cluster.point_count = static_cast<std::uint32_t>(candidate.point_count);
      cluster.dynamic = true;
      out.clusters.push_back(cluster);
    }

    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string map_topic_;
  std::string sensor_frame_;
  std::string odom_topic_;

  bool use_static_map_{true};
  bool require_static_map_{true};
  long occupied_threshold_{65};
  double static_inflation_radius_{0.0};
  double cluster_tolerance_{0.0};
  long min_cluster_points_{0};
  long max_cluster_points_{0};
  long max_clusters_{0};
  double association_distance_{0.0};
  double min_dynamic_displacement_{0.0};
  double min_dynamic_speed_{0.0};
  long min_dynamic_hits_{0};
  long max_track_misses_{0};
  double min_cluster_size_xy_{0.0};
  double max_cluster_size_xy_{0.0};
  double wall_sliver_min_length_{0.0};
  double wall_sliver_max_length_{0.0};
  double wall_sliver_max_width_{0.0};
  double static_map_occupied_ratio_threshold_{0.0};
  double static_map_large_cluster_min_length_{0.0};
  double static_map_large_cluster_max_length_{0.0};
  double static_map_large_cluster_max_width_{0.0};
  double static_shape_min_length_{0.0};
  double static_shape_max_length_{0.0};
  double static_shape_max_width_{0.0};
  bool shadow_management_enabled_{true};
  double shadow_tf_timeout_sec_{0.0};
  double shadow_persistence_sec_{0.0};
  double shadow_angle_margin_rad_{0.0};
  double shadow_range_padding_{0.0};
  double shadow_max_range_{0.0};
  bool ego_velocity_rejection_{true};
  double ego_speed_ratio_threshold_{0.0};
  double ego_min_robot_speed_{0.0};
  double ego_artifact_min_speed_ratio_{0.0};
  double ego_artifact_max_speed_ratio_{0.0};
  double ego_artifact_direction_cosine_{0.0};
  bool filter_out_of_map_{true};
  long min_consecutive_dynamic_hits_{0};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::map<int, PreviousCluster> previous_;
  std::vector<ShadowCone> shadow_cones_;
  int next_id_{1};

  double ego_vx_{0.0};
  double ego_vy_{0.0};
  double ego_speed_{0.0};
  double ego_yaw_rate_{0.0};
  Point2 ego_map_velocity_{Point2::Zero()};
  double ego_map_speed_{0.0};
  std::optional<Point2> last_sensor_xy_;
  std::optional<double> last_sensor_stamp_;
  double last_shadow_tf_warn_time_sec_{0.0};
  int startup_shadow_tf_failures_{0};
  const int startup_shadow_tf_grace_failures_{10};
  bool has_shadow_tf_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<robot_common_interfaces::msg::DynamicObstacleClusterArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

}  // namespace robot_nav::dynamic_obstacles

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_nav::dynamic_obstacles::DynamicClusterDetector>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "dynamic_cluster_detector crashed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
