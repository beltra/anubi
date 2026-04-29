#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot_nav::dynamic_obstacles
{

using Point2 = Eigen::Vector2d;
using Point3 = Eigen::Vector3d;
using TrackVector = Eigen::Matrix<double, 5, 1>;
using TrackMatrix = Eigen::Matrix<double, 5, 5>;

struct ShadowCone
{
  Point2 origin{Point2::Zero()};
  double bearing{0.0};
  double half_angle{0.0};
  double min_range{0.0};
  double max_range{0.0};
  double expires_at{0.0};
};

double stamp_seconds(const builtin_interfaces::msg::Time & stamp);
builtin_interfaces::msg::Time stamp_plus_seconds(
  const builtin_interfaces::msg::Time & stamp,
  double seconds);

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);
double normalize_angle(double angle);

geometry_msgs::msg::Point point_msg(const Point3 & values);
geometry_msgs::msg::Vector3 vector_msg(const Point3 & values);
geometry_msgs::msg::Pose pose_from_xy_yaw(double x, double y, double z, double yaw);

sensor_msgs::msg::PointCloud2 cloud_from_points(
  const std_msgs::msg::Header & header,
  const std::vector<Point3> & points);
std::vector<Point3> points_from_cloud(const sensor_msgs::msg::PointCloud2 & msg);

Eigen::Matrix4d transform_to_matrix(const geometry_msgs::msg::TransformStamped & transform);
std::vector<Point3> transform_points(
  const std::vector<Point3> & points,
  const Eigen::Matrix4d & matrix);
std::vector<Point3> filter_height(
  const std::vector<Point3> & points,
  double min_z,
  double max_z);
std::vector<Point3> voxel_downsample(
  const std::vector<Point3> & points,
  double voxel_size);
std::vector<Point3> oriented_footprint_points(
  double x,
  double y,
  double yaw,
  double length,
  double width,
  double resolution,
  double z,
  double padding = 0.0);

std::vector<std::vector<std::size_t>> euclidean_clusters(
  const std::vector<Point3> & points,
  double tolerance,
  int min_points,
  int max_points,
  int max_clusters);

ShadowCone shadow_cone_from_obstacle(
  const Point2 & sensor_xy,
  const Point2 & obstacle_xy,
  const Point2 & obstacle_size_xy,
  double stamp,
  double angle_margin,
  double range_padding,
  double max_range,
  double persistence,
  bool * valid = nullptr);
std::vector<bool> points_in_shadow_cones(
  const std::vector<Point3> & points,
  const std::vector<ShadowCone> & cones);
bool velocity_matches_ego_motion(
  const Point2 & velocity_xy,
  const Point2 & ego_velocity_xy,
  double min_ego_speed,
  double min_speed_ratio,
  double max_speed_ratio,
  double direction_cosine);

TrackVector ctrv_predict_state(
  const TrackVector & x,
  double dt,
  double yaw_rate_eps);
TrackMatrix ctrv_jacobian(
  const TrackVector & x,
  double dt,
  double yaw_rate_eps);

bool track_is_publishable(
  int hits,
  int misses,
  int confirmation_hits,
  int max_publish_misses,
  double confidence,
  double min_confidence);

}  // namespace robot_nav::dynamic_obstacles
