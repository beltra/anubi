#include "robot_nav/dynamic_obstacles/common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace robot_nav::dynamic_obstacles
{
namespace
{

struct Int2
{
  long x{0};
  long y{0};

  bool operator==(const Int2 & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct Int2Hash
{
  std::size_t operator()(const Int2 & key) const
  {
    const auto hx = std::hash<long>{}(key.x);
    const auto hy = std::hash<long>{}(key.y);
    return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2));
  }
};

struct Int3
{
  long x{0};
  long y{0};
  long z{0};

  bool operator==(const Int3 & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct Int3Hash
{
  std::size_t operator()(const Int3 & key) const
  {
    const auto hx = std::hash<long>{}(key.x);
    const auto hy = std::hash<long>{}(key.y);
    const auto hz = std::hash<long>{}(key.z);
    return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2)) ^
           (hz + 0x9e3779b9 + (hy << 6) + (hy >> 2));
  }
};

struct FieldInfo
{
  std::uint32_t offset{0};
  std::uint8_t datatype{0};
  bool found{false};
};

FieldInfo find_field(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return {field.offset, field.datatype, true};
    }
  }
  return {};
}

template<typename T>
T read_unaligned(const std::uint8_t * ptr)
{
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

double read_field_value(const std::uint8_t * point, const FieldInfo & field)
{
  const std::uint8_t * ptr = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(read_unaligned<std::int8_t>(ptr));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(read_unaligned<std::uint8_t>(ptr));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(read_unaligned<std::int16_t>(ptr));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(read_unaligned<std::uint16_t>(ptr));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(read_unaligned<std::int32_t>(ptr));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(read_unaligned<std::uint32_t>(ptr));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(read_unaligned<float>(ptr));
    case sensor_msgs::msg::PointField::FLOAT64:
      return read_unaligned<double>(ptr);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

Eigen::Matrix3d quaternion_to_matrix(const geometry_msgs::msg::Quaternion & q)
{
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  const double w = q.w;
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;

  Eigen::Matrix3d out;
  out <<
    1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
    2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
    2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy);
  return out;
}

}  // namespace

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time stamp_plus_seconds(
  const builtin_interfaces::msg::Time & stamp,
  double seconds)
{
  const std::int64_t base_ns =
    static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(stamp.nanosec);
  const std::int64_t offset_ns = static_cast<std::int64_t>(seconds * 1.0e9);
  const std::int64_t total_ns = base_ns + offset_ns;

  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(total_ns / 1000000000LL);
  out.nanosec = static_cast<std::uint32_t>(total_ns % 1000000000LL);
  return out;
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Point point_msg(const Point3 & values)
{
  geometry_msgs::msg::Point out;
  out.x = values.x();
  out.y = values.y();
  out.z = values.z();
  return out;
}

geometry_msgs::msg::Vector3 vector_msg(const Point3 & values)
{
  geometry_msgs::msg::Vector3 out;
  out.x = values.x();
  out.y = values.y();
  out.z = values.z();
  return out;
}

geometry_msgs::msg::Pose pose_from_xy_yaw(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = yaw_to_quaternion(yaw);
  return pose;
}

sensor_msgs::msg::PointCloud2 cloud_from_points(
  const std_msgs::msg::Header & header,
  const std::vector<Point3> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(cloud, "z");
  for (const auto & point : points) {
    *x_iter = static_cast<float>(point.x());
    *y_iter = static_cast<float>(point.y());
    *z_iter = static_cast<float>(point.z());
    ++x_iter;
    ++y_iter;
    ++z_iter;
  }
  return cloud;
}

std::vector<Point3> points_from_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (msg.width * msg.height == 0U || msg.point_step == 0U) {
    return {};
  }

  const auto x_field = find_field(msg, "x");
  const auto y_field = find_field(msg, "y");
  const auto z_field = find_field(msg, "z");
  if (!x_field.found || !y_field.found || !z_field.found) {
    return {};
  }

  const std::size_t point_count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  std::vector<Point3> points;
  points.reserve(point_count);

  const auto * raw = msg.data.data();
  for (std::size_t idx = 0; idx < point_count; ++idx) {
    const auto * point = raw + idx * msg.point_step;
    const double x = read_field_value(point, x_field);
    const double y = read_field_value(point, y_field);
    const double z = read_field_value(point, z_field);
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points.emplace_back(x, y, z);
    }
  }
  return points;
}

Eigen::Matrix4d transform_to_matrix(const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.block<3, 3>(0, 0) = quaternion_to_matrix(transform.transform.rotation);
  out(0, 3) = transform.transform.translation.x;
  out(1, 3) = transform.transform.translation.y;
  out(2, 3) = transform.transform.translation.z;
  return out;
}

std::vector<Point3> transform_points(
  const std::vector<Point3> & points,
  const Eigen::Matrix4d & matrix)
{
  std::vector<Point3> out;
  out.reserve(points.size());
  for (const auto & point : points) {
    Eigen::Vector4d hom(point.x(), point.y(), point.z(), 1.0);
    const Eigen::Vector4d result = matrix * hom;
    out.emplace_back(result.x(), result.y(), result.z());
  }
  return out;
}

std::vector<Point3> filter_height(
  const std::vector<Point3> & points,
  double min_z,
  double max_z)
{
  std::vector<Point3> out;
  out.reserve(points.size());
  for (const auto & point : points) {
    if (
      std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z()) &&
      point.z() >= min_z && point.z() <= max_z)
    {
      out.push_back(point);
    }
  }
  return out;
}

std::vector<Point3> voxel_downsample(
  const std::vector<Point3> & points,
  double voxel_size)
{
  if (points.empty() || voxel_size <= 0.0) {
    return points;
  }

  std::unordered_set<Int3, Int3Hash> seen;
  seen.reserve(points.size());

  std::vector<Point3> out;
  out.reserve(points.size());
  for (const auto & point : points) {
    const Int3 key{
      static_cast<long>(std::floor(point.x() / voxel_size)),
      static_cast<long>(std::floor(point.y() / voxel_size)),
      static_cast<long>(std::floor(point.z() / voxel_size)),
    };
    if (seen.insert(key).second) {
      out.push_back(point);
    }
  }
  return out;
}

std::vector<Point3> oriented_footprint_points(
  double x,
  double y,
  double yaw,
  double length,
  double width,
  double resolution,
  double z,
  double padding)
{
  const double step = std::max(0.05, resolution);
  const double half_length = std::max(0.05, 0.5 * length + padding);
  const double half_width = std::max(0.05, 0.5 * width + padding);
  const int nx = std::max(1, static_cast<int>(std::ceil(half_length / step)));
  const int ny = std::max(1, static_cast<int>(std::ceil(half_width / step)));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  std::vector<Point3> points;
  points.reserve(static_cast<std::size_t>((2 * nx + 1) * (2 * ny + 1)));
  for (int ix = -nx; ix <= nx; ++ix) {
    const double local_x = std::max(-half_length, std::min(half_length, ix * step));
    for (int iy = -ny; iy <= ny; ++iy) {
      const double local_y = std::max(-half_width, std::min(half_width, iy * step));
      const double px = x + cos_yaw * local_x - sin_yaw * local_y;
      const double py = y + sin_yaw * local_x + cos_yaw * local_y;
      points.emplace_back(px, py, z);
    }
  }
  return points;
}

std::vector<std::vector<std::size_t>> euclidean_clusters(
  const std::vector<Point3> & points,
  double tolerance,
  int min_points,
  int max_points,
  int max_clusters)
{
  if (points.size() < static_cast<std::size_t>(min_points)) {
    return {};
  }

  const double cell = std::max(tolerance, 1.0e-3);
  std::unordered_map<Int2, std::vector<std::size_t>, Int2Hash> cells;
  cells.reserve(points.size());
  for (std::size_t idx = 0; idx < points.size(); ++idx) {
    const Int2 key{
      static_cast<long>(std::floor(points[idx].x() / cell)),
      static_cast<long>(std::floor(points[idx].y() / cell)),
    };
    cells[key].push_back(idx);
  }

  const double tol_sq = tolerance * tolerance;
  std::vector<bool> visited(points.size(), false);
  std::vector<std::vector<std::size_t>> clusters;
  clusters.reserve(static_cast<std::size_t>(std::max(0, max_clusters)));

  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }

    std::vector<std::size_t> queue{seed};
    visited[seed] = true;
    std::vector<std::size_t> members;

    while (!queue.empty()) {
      const std::size_t current = queue.back();
      queue.pop_back();
      members.push_back(current);

      const Int2 key{
        static_cast<long>(std::floor(points[current].x() / cell)),
        static_cast<long>(std::floor(points[current].y() / cell)),
      };
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const Int2 neighbor{key.x + dx, key.y + dy};
          const auto it = cells.find(neighbor);
          if (it == cells.end()) {
            continue;
          }
          for (const auto candidate : it->second) {
            if (visited[candidate]) {
              continue;
            }
            const Eigen::Vector2d delta =
              points[candidate].head<2>() - points[current].head<2>();
            if (delta.squaredNorm() <= tol_sq) {
              visited[candidate] = true;
              queue.push_back(candidate);
            }
          }
        }
      }
    }

    if (
      static_cast<int>(members.size()) >= min_points &&
      static_cast<int>(members.size()) <= max_points)
    {
      clusters.push_back(std::move(members));
      if (static_cast<int>(clusters.size()) >= max_clusters) {
        break;
      }
    }
  }

  return clusters;
}

ShadowCone shadow_cone_from_obstacle(
  const Point2 & sensor_xy,
  const Point2 & obstacle_xy,
  const Point2 & obstacle_size_xy,
  double stamp,
  double angle_margin,
  double range_padding,
  double max_range,
  double persistence,
  bool * valid)
{
  const Point2 delta = obstacle_xy - sensor_xy;
  const double distance = delta.norm();
  if (distance <= 1.0e-3) {
    if (valid != nullptr) {
      *valid = false;
    }
    return {};
  }

  const double half_extent = 0.5 * obstacle_size_xy.cwiseMax(0.0).norm();
  const double padded_extent = std::max(0.01, half_extent + std::max(0.0, range_padding));
  const double ratio = std::min(0.95, padded_extent / distance);
  const double half_angle = std::asin(ratio) + std::max(0.0, angle_margin);
  const double min_range = distance + padded_extent;

  if (valid != nullptr) {
    *valid = true;
  }
  ShadowCone cone;
  cone.origin = sensor_xy;
  cone.bearing = std::atan2(delta.y(), delta.x());
  cone.half_angle = half_angle;
  cone.min_range = min_range;
  cone.max_range = std::max(min_range, max_range);
  cone.expires_at = stamp + std::max(0.0, persistence);
  return cone;
}

std::vector<bool> points_in_shadow_cones(
  const std::vector<Point3> & points,
  const std::vector<ShadowCone> & cones)
{
  std::vector<bool> mask(points.size(), false);
  if (points.empty() || cones.empty()) {
    return mask;
  }

  for (std::size_t idx = 0; idx < points.size(); ++idx) {
    const Point2 xy = points[idx].head<2>();
    for (const auto & cone : cones) {
      const Point2 delta = xy - cone.origin;
      const double range = delta.norm();
      const double bearing = std::atan2(delta.y(), delta.x());
      const double angle_error = std::abs(
        std::atan2(std::sin(bearing - cone.bearing), std::cos(bearing - cone.bearing)));
      if (
        range >= cone.min_range &&
        range <= cone.max_range &&
        angle_error <= cone.half_angle)
      {
        mask[idx] = true;
        break;
      }
    }
  }
  return mask;
}

bool velocity_matches_ego_motion(
  const Point2 & velocity_xy,
  const Point2 & ego_velocity_xy,
  double min_ego_speed,
  double min_speed_ratio,
  double max_speed_ratio,
  double direction_cosine)
{
  const double speed = velocity_xy.norm();
  const double ego_speed = ego_velocity_xy.norm();
  if (speed <= 1.0e-6 || ego_speed < min_ego_speed) {
    return false;
  }

  const double ratio = speed / ego_speed;
  if (ratio < min_speed_ratio || ratio > max_speed_ratio) {
    return false;
  }

  const double cosine = velocity_xy.dot(ego_velocity_xy) / (speed * ego_speed);
  return std::abs(cosine) >= direction_cosine;
}

TrackVector ctrv_predict_state(
  const TrackVector & x,
  double dt,
  double yaw_rate_eps)
{
  TrackVector out = x;
  const double px = x(0);
  const double py = x(1);
  const double yaw = x(2);
  const double speed = x(3);
  const double yaw_rate = x(4);

  if (std::abs(yaw_rate) < yaw_rate_eps) {
    out(0) = px + speed * dt * std::cos(yaw);
    out(1) = py + speed * dt * std::sin(yaw);
  } else {
    const double yaw_next = yaw + yaw_rate * dt;
    out(0) = px + (speed / yaw_rate) * (std::sin(yaw_next) - std::sin(yaw));
    out(1) = py + (speed / yaw_rate) * (-std::cos(yaw_next) + std::cos(yaw));
  }

  out(2) = normalize_angle(yaw + yaw_rate * dt);
  out(3) = speed;
  out(4) = yaw_rate;
  return out;
}

TrackMatrix ctrv_jacobian(
  const TrackVector & x,
  double dt,
  double yaw_rate_eps)
{
  const double yaw = x(2);
  const double speed = x(3);
  const double yaw_rate = x(4);

  TrackMatrix f = TrackMatrix::Identity();
  if (std::abs(yaw_rate) < yaw_rate_eps) {
    f(0, 2) = -speed * dt * std::sin(yaw);
    f(0, 3) = dt * std::cos(yaw);
    f(1, 2) = speed * dt * std::cos(yaw);
    f(1, 3) = dt * std::sin(yaw);
  } else {
    const double yaw_next = yaw + yaw_rate * dt;
    f(0, 2) = (speed / yaw_rate) * (std::cos(yaw_next) - std::cos(yaw));
    f(0, 3) = (std::sin(yaw_next) - std::sin(yaw)) / yaw_rate;
    f(0, 4) =
      speed * dt * std::cos(yaw_next) / yaw_rate -
      speed * (std::sin(yaw_next) - std::sin(yaw)) / (yaw_rate * yaw_rate);
    f(1, 2) = (speed / yaw_rate) * (std::sin(yaw_next) - std::sin(yaw));
    f(1, 3) = (-std::cos(yaw_next) + std::cos(yaw)) / yaw_rate;
    f(1, 4) =
      speed * dt * std::sin(yaw_next) / yaw_rate -
      speed * (-std::cos(yaw_next) + std::cos(yaw)) / (yaw_rate * yaw_rate);
  }
  f(2, 4) = dt;
  return f;
}

bool track_is_publishable(
  int hits,
  int misses,
  int confirmation_hits,
  int max_publish_misses,
  double confidence,
  double min_confidence)
{
  return
    hits >= confirmation_hits &&
    misses <= max_publish_misses &&
    confidence >= min_confidence;
}

}  // namespace robot_nav::dynamic_obstacles
