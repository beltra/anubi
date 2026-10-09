#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string format_double(double value, int precision)
{
  if (std::isnan(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed, std::ios::floatfield);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

KeyValue kv(const std::string & key, const std::string & value)
{
  KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3
{
  std::array<double, 9> m{};

  Vec3 operator*(const Vec3 & v) const
  {
    return {
      m[0] * v.x + m[1] * v.y + m[2] * v.z,
      m[3] * v.x + m[4] * v.y + m[5] * v.z,
      m[6] * v.x + m[7] * v.y + m[8] * v.z,
    };
  }
};

Mat3 transpose(const Mat3 & in)
{
  return {{
    in.m[0], in.m[3], in.m[6],
    in.m[1], in.m[4], in.m[7],
    in.m[2], in.m[5], in.m[8],
  }};
}

Mat3 rotation_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {{
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  }};
}

std::pair<double, double> roll_pitch_from_imu(const geometry_msgs::msg::Quaternion & q)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, sinp)));
  return {roll, pitch};
}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
  };
}

Vec3 add(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 normalize(const Vec3 & a)
{
  const double n = std::sqrt(dot(a, a));
  if (n < 1e-12) {
    return {0.0, 0.0, 1.0};
  }
  return {a.x / n, a.y / n, a.z / n};
}

Mat3 identity3()
{
  return {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
}

Mat3 mat_mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 c{};
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += a.m[r * 3 + k] * b.m[k * 3 + col];
      }
      c.m[r * 3 + col] = s;
    }
  }
  return c;
}

Mat3 skew(const Vec3 & v)
{
  return {{0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0}};
}

Mat3 rotation_axis_angle(const Vec3 & axis, double angle)
{
  const Vec3 a = normalize(axis);
  const Mat3 k = skew(a);
  const Mat3 kk = mat_mul(k, k);
  const double sn = std::sin(angle);
  const double cs = std::cos(angle);
  Mat3 r = identity3();
  for (int i = 0; i < 9; ++i) {
    r.m[i] += sn * k.m[i] + (1.0 - cs) * kk.m[i];
  }
  return r;
}

// Minimal (shortest-arc) rotation mapping unit-ish vector `from` onto `to`.
Mat3 rotation_between(const Vec3 & from, const Vec3 & to)
{
  const Vec3 a = normalize(from);
  const Vec3 b = normalize(to);
  const Vec3 v = cross(a, b);
  const double s = std::sqrt(dot(v, v));
  const double c = dot(a, b);
  if (s < 1e-8) {
    if (c > 0.0) {
      return identity3();  // already aligned
    }
    // Antiparallel: rotate 180 deg about any axis perpendicular to a.
    const Vec3 perp = std::abs(a.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    const Mat3 k = skew(normalize(cross(a, perp)));
    const Mat3 kk = mat_mul(k, k);
    Mat3 r = identity3();
    for (int i = 0; i < 9; ++i) {
      r.m[i] += 2.0 * kk.m[i];
    }
    return r;
  }
  const Mat3 k = skew(v);
  const Mat3 kk = mat_mul(k, k);
  const double f = (1.0 - c) / (s * s);
  Mat3 r = identity3();
  for (int i = 0; i < 9; ++i) {
    r.m[i] += k.m[i] + f * kk.m[i];
  }
  return r;
}

struct FieldInfo
{
  uint32_t offset{0};
  uint8_t datatype{0};
  std::string name;
  bool found{false};
};

FieldInfo find_field(const PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return {field.offset, field.datatype, field.name, true};
    }
  }
  return {};
}

template<typename T>
T read_unaligned(const uint8_t * ptr)
{
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

template<typename T>
void write_unaligned(uint8_t * ptr, T value)
{
  std::memcpy(ptr, &value, sizeof(T));
}

double read_field_value(const uint8_t * point, const FieldInfo & field)
{
  const uint8_t * ptr = point + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      return static_cast<double>(read_unaligned<int8_t>(ptr));
    case PointField::UINT8:
      return static_cast<double>(read_unaligned<uint8_t>(ptr));
    case PointField::INT16:
      return static_cast<double>(read_unaligned<int16_t>(ptr));
    case PointField::UINT16:
      return static_cast<double>(read_unaligned<uint16_t>(ptr));
    case PointField::INT32:
      return static_cast<double>(read_unaligned<int32_t>(ptr));
    case PointField::UINT32:
      return static_cast<double>(read_unaligned<uint32_t>(ptr));
    case PointField::FLOAT32:
      return static_cast<double>(read_unaligned<float>(ptr));
    case PointField::FLOAT64:
      return read_unaligned<double>(ptr);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

bool write_float_field(uint8_t * point, const FieldInfo & field, float value)
{
  uint8_t * ptr = point + field.offset;
  switch (field.datatype) {
    case PointField::FLOAT32:
      write_unaligned<float>(ptr, value);
      return true;
    case PointField::FLOAT64:
      write_unaligned<double>(ptr, static_cast<double>(value));
      return true;
    default:
      return false;
  }
}

bool is_float_field(const FieldInfo & field)
{
  return field.datatype == PointField::FLOAT32 || field.datatype == PointField::FLOAT64;
}

}  // namespace

class DistortionCorrector : public rclcpp::Node
{
public:
  DistortionCorrector()
  : Node("distortion_corrector")
  {
    declare_parameters();
    load_parameters();

    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    publisher_ = create_publisher<PointCloud2>(output_topic_, cloud_qos);
    if (publish_diagnostics_) {
      diagnostics_pub_ = create_publisher<DiagnosticArray>(diagnostics_topic_, 10);
    }
    cloud_sub_ = create_subscription<PointCloud2>(
      input_topic_, cloud_qos,
      std::bind(&DistortionCorrector::cloud_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DistortionCorrector::odom_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DistortionCorrector::imu_callback, this, std::placeholders::_1));

    if (head_imu_leveling_) {
      const std::string lev_topic = leveling_imu_topic_.empty() ? imu_topic_ : leveling_imu_topic_;
      leveling_imu_sub_ = create_subscription<Imu>(
        lev_topic, rclcpp::SensorDataQoS(),
        std::bind(&DistortionCorrector::leveling_imu_callback, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "head_imu_leveling ON: complementary filter on %s (gain=%.3f)",
        lev_topic.c_str(), head_imu_comp_gain_);
    }

    RCLCPP_INFO(
      get_logger(),
      "LiDAR processor: %s -> %s (deskew=%s, gravity_align=%s->%s, "
      "self_filter=%s, mount_rotation_mode=%s, diagnostics=%s)",
      input_topic_.c_str(), output_topic_.c_str(), deskew_enabled_ ? "true" : "false",
      gravity_align_ ? "true" : "false", gravity_align_frame_.c_str(),
      filter_self_ ? "true" : "false", mount_rotation_mode_.c_str(),
      publish_diagnostics_ ? "true" : "false");
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("input_topic", "/lidar/points");
    declare_parameter<std::string>("output_topic", "/lidar/points_undistorted");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<bool>("deskew_enabled", false);
    declare_parameter<bool>("use_imu", true);
    declare_parameter<bool>("use_twist", true);
    declare_parameter<bool>("use_3d_distortion_correction", true);
    declare_parameter<double>("scan_period", 0.1);
    declare_parameter<std::vector<double>>(
      "sensor_mount_rpy", {0.0, -2.88000012056847, 0.0});
    declare_parameter<std::string>("mount_rotation_mode", "inverse");
    declare_parameter<std::string>("timestamp_field_name", "");
    declare_parameter<std::vector<std::string>>(
      "timestamp_field_candidates", {"time_stamp", "timestamp", "time", "t"});
    declare_parameter<std::string>("timestamp_unit", "auto");
    declare_parameter<std::string>("fallback_mode", "azimuth");
    declare_parameter<std::string>("azimuth_direction", "cw");
    // Deskew reference instant: "start" (earliest point), "median", or "mean".
    // "median" matches DLIO's deskewed cloud so the two align in yaw during turns.
    declare_parameter<std::string>("deskew_reference", "start");
    declare_parameter<double>("imu_timeout_sec", 0.25);
    declare_parameter<double>("odom_timeout_sec", 0.25);
    declare_parameter<std::string>("output_stamp_mode", "input");
    declare_parameter<bool>("gravity_align", true);
    declare_parameter<std::string>("gravity_align_frame", "lidar_stabilized");
    // Head-IMU leveling: level the cloud from the sensor's OWN IMU (Mid-360),
    // which physically rides on the head and observes head-tilt excursions the
    // static mount and the pelvis IMU cannot (there is no neck encoder). A
    // complementary filter on that IMU (gyro for fast tilt, low-passed accel for
    // drift-free gravity) estimates the gravity direction; the leveling rotation
    // then maps the estimated floor normal to +z. Overrides the orientation-based
    // gravity_align path when enabled. leveling_imu_topic must carry gyro (rad/s)
    // and accel; empty falls back to imu_topic. head_imu_ref_up / cloud_ref_up are
    // the still-segment calibration (head-IMU gravity-up unit vector and the
    // matching cloud floor-normal-up); their shortest-arc rotation is the
    // head-IMU->cloud extrinsic used to map the live gravity estimate.
    declare_parameter<bool>("head_imu_leveling", false);
    // Floor-plane leveling: fit the ground plane in each sweep and rotate its
    // normal to +z, so the floor is level BY CONSTRUCTION -- no IMU, no gyro bias,
    // no extrinsic calibration, and robust to head tilt. Takes precedence over
    // head_imu_leveling. Caveats: it levels to the VISIBLE ground (wrong on a
    // ramp/slope -- it would make the ramp horizontal, not gravity-level), and it
    // needs enough floor points, so it holds the last good normal when the fit
    // fails. The normal is low-passed across sweeps to reject fit jitter.
    declare_parameter<bool>("floor_plane_leveling", false);
    declare_parameter<double>("floor_z_min", -1.6);
    declare_parameter<double>("floor_z_max", -0.7);
    declare_parameter<double>("floor_r_min", 0.5);
    declare_parameter<double>("floor_r_max", 6.0);
    declare_parameter<int>("floor_min_points", 250);
    declare_parameter<double>("floor_inlier_thresh", 0.05);
    declare_parameter<double>("floor_normal_lowpass", 0.2);
    declare_parameter<double>("floor_max_tilt_deg", 12.0);
    declare_parameter<std::string>("leveling_imu_topic", "");
    declare_parameter<std::vector<double>>("head_imu_ref_up", {0.0, 0.0, -1.0});
    declare_parameter<std::vector<double>>("cloud_ref_up", {0.0, 0.0, 1.0});
    // The still-pose calibration fixes only 2 of 3 extrinsic DOF; this pins the
    // remaining yaw-about-gravity DOF (fit by minimizing run-wide floor tilt).
    declare_parameter<double>("head_imu_extrinsic_yaw_deg", 0.0);
    declare_parameter<double>("head_imu_comp_gain", 0.01);
    declare_parameter<double>("head_imu_max_dt", 0.1);
    // Gyro bias (rad/s) subtracted before propagation; the Mid-360 gyro carries a
    // large bias (~0.24 rad/s) that otherwise drags the estimate ~8 deg off level.
    // Left at zero, it is auto-estimated by averaging the first
    // head_imu_bias_init_sec of leveling-IMU samples (the launch holds the robot
    // still during the nav-delay for DLIO gravity calibration -- same window).
    declare_parameter<std::vector<double>>("head_imu_gyro_bias", {0.0, 0.0, 0.0});
    declare_parameter<double>("head_imu_bias_init_sec", 2.0);
    // Online gyro-bias adaptation (Mahony-style integral feedback from the
    // accelerometer): the Mid-360 gyro bias DRIFTS ~0.08 rad/s over minutes
    // (warmup), so a one-shot init is not enough -- it would leave a ~3 deg
    // steady-state leveling error. Adapts only on near-gravity samples
    // (|accel|-g within tolerance) so gait acceleration does not corrupt it.
    declare_parameter<double>("head_imu_bias_learn_rate", 0.02);
    declare_parameter<double>("head_imu_bias_accel_tol", 1.0);
    // Scale on the IMU roll/pitch injected by gravity_align. 1.0 = full IMU
    // leveling; 0.0 = static mount righting only (no IMU attitude enters the
    // cloud; the align rotation degenerates to the mount rotation, so the
    // output frame equals the body-oriented frame at the sensor). On robots
    // whose head is already gait-stabilized (G1), full pelvis-IMU leveling
    // INJECTS attitude the sensor never had.
    declare_parameter<double>("gravity_align_attenuation", 1.0);
    // Warn when the IMU sample matched to a cloud stamp is much older than
    // now(): stale cloud stamps silently pull old samples out of the history
    // buffer (imu_timeout_sec never fires inside the buffered past).
    declare_parameter<double>("imu_stale_warn_sec", 0.5);
    declare_parameter<bool>("filter_self", false);
    declare_parameter<double>("robot_min_x", -0.40);
    declare_parameter<double>("robot_max_x", 0.36);
    declare_parameter<double>("robot_min_y", -0.20);
    declare_parameter<double>("robot_max_y", 0.20);
    declare_parameter<double>("robot_min_z", -0.25);
    declare_parameter<double>("robot_max_z", 0.10);
    declare_parameter<double>("min_radius", 0.12);
    declare_parameter<double>("sensor_x_in_base", 0.28945);
    declare_parameter<double>("sensor_y_in_base", 0.0);
    declare_parameter<double>("sensor_z_in_base", -0.046825);
    declare_parameter<bool>("publish_diagnostics", false);
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics/lidar_correction");
  }

  void load_parameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    deskew_enabled_ = get_parameter("deskew_enabled").as_bool();
    use_imu_ = get_parameter("use_imu").as_bool();
    use_twist_ = get_parameter("use_twist").as_bool();
    use_3d_ = get_parameter("use_3d_distortion_correction").as_bool();
    scan_period_ = get_parameter("scan_period").as_double();
    mount_rotation_mode_ = get_parameter("mount_rotation_mode").as_string();
    if (mount_rotation_mode_ != "direct" && mount_rotation_mode_ != "inverse") {
      RCLCPP_WARN(
        get_logger(), "Unsupported mount_rotation_mode '%s'; using 'inverse'",
        mount_rotation_mode_.c_str());
      mount_rotation_mode_ = "inverse";
    }
    timestamp_field_name_ = get_parameter("timestamp_field_name").as_string();
    timestamp_field_candidates_ = get_parameter("timestamp_field_candidates").as_string_array();
    timestamp_unit_ = get_parameter("timestamp_unit").as_string();
    fallback_mode_ = get_parameter("fallback_mode").as_string();
    azimuth_direction_ = get_parameter("azimuth_direction").as_string();
    deskew_reference_ = get_parameter("deskew_reference").as_string();
    imu_timeout_sec_ = get_parameter("imu_timeout_sec").as_double();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    output_stamp_mode_ = get_parameter("output_stamp_mode").as_string();
    gravity_align_ = get_parameter("gravity_align").as_bool();
    gravity_align_frame_ = get_parameter("gravity_align_frame").as_string();
    head_imu_leveling_ = get_parameter("head_imu_leveling").as_bool();
    floor_plane_leveling_ = get_parameter("floor_plane_leveling").as_bool();
    floor_z_min_ = get_parameter("floor_z_min").as_double();
    floor_z_max_ = get_parameter("floor_z_max").as_double();
    floor_r_min_ = get_parameter("floor_r_min").as_double();
    floor_r_max_ = get_parameter("floor_r_max").as_double();
    floor_min_points_ =
      static_cast<size_t>(std::max<int64_t>(0, get_parameter("floor_min_points").as_int()));
    floor_inlier_thresh_ = get_parameter("floor_inlier_thresh").as_double();
    floor_normal_lowpass_ = std::max(
      0.0, std::min(1.0, get_parameter("floor_normal_lowpass").as_double()));
    floor_max_tilt_cos_ = std::cos(get_parameter("floor_max_tilt_deg").as_double() * kPi / 180.0);
    leveling_imu_topic_ = get_parameter("leveling_imu_topic").as_string();
    head_imu_comp_gain_ = std::max(0.0, std::min(1.0, get_parameter("head_imu_comp_gain").as_double()));
    head_imu_max_dt_ = get_parameter("head_imu_max_dt").as_double();
    {
      const auto ru = get_parameter("head_imu_ref_up").as_double_array();
      const auto cu = get_parameter("cloud_ref_up").as_double_array();
      const Vec3 ref_up{
        ru.size() > 0 ? ru[0] : 0.0, ru.size() > 1 ? ru[1] : 0.0, ru.size() > 2 ? ru[2] : -1.0};
      const Vec3 cloud_up{
        cu.size() > 0 ? cu[0] : 0.0, cu.size() > 1 ? cu[1] : 0.0, cu.size() > 2 ? cu[2] : 1.0};
      // Shortest-arc head-IMU-up -> cloud-up (2 DOF), then the fitted
      // yaw-about-gravity DOF pins the full extrinsic.
      const double yaw = get_parameter("head_imu_extrinsic_yaw_deg").as_double() * kPi / 180.0;
      R_head_imu_to_cloud_ = mat_mul(
        rotation_between(ref_up, cloud_up), rotation_axis_angle(ref_up, yaw));
    }
    {
      const auto gb = get_parameter("head_imu_gyro_bias").as_double_array();
      head_imu_gyro_bias_ = {
        gb.size() > 0 ? gb[0] : 0.0, gb.size() > 1 ? gb[1] : 0.0, gb.size() > 2 ? gb[2] : 0.0};
      head_imu_bias_init_sec_ = get_parameter("head_imu_bias_init_sec").as_double();
      // Auto-estimate only when no explicit bias was provided.
      bias_estimated_ =
        head_imu_gyro_bias_.x != 0.0 || head_imu_gyro_bias_.y != 0.0 ||
        head_imu_gyro_bias_.z != 0.0 || head_imu_bias_init_sec_ <= 0.0;
    }
    head_imu_bias_learn_rate_ = get_parameter("head_imu_bias_learn_rate").as_double();
    head_imu_bias_accel_tol_ = get_parameter("head_imu_bias_accel_tol").as_double();
    gravity_align_attenuation_ = std::max(
      0.0, std::min(1.0, get_parameter("gravity_align_attenuation").as_double()));
    imu_stale_warn_sec_ = get_parameter("imu_stale_warn_sec").as_double();
    filter_self_ = get_parameter("filter_self").as_bool();
    robot_min_x_ = get_parameter("robot_min_x").as_double();
    robot_max_x_ = get_parameter("robot_max_x").as_double();
    robot_min_y_ = get_parameter("robot_min_y").as_double();
    robot_max_y_ = get_parameter("robot_max_y").as_double();
    robot_min_z_ = get_parameter("robot_min_z").as_double();
    robot_max_z_ = get_parameter("robot_max_z").as_double();
    min_radius_ = get_parameter("min_radius").as_double();
    sensor_x_in_base_ = get_parameter("sensor_x_in_base").as_double();
    sensor_y_in_base_ = get_parameter("sensor_y_in_base").as_double();
    sensor_z_in_base_ = get_parameter("sensor_z_in_base").as_double();
    publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
    diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

    const auto rpy = get_parameter("sensor_mount_rpy").as_double_array();
    const double roll = rpy.size() > 0 ? rpy[0] : 0.0;
    const double pitch = rpy.size() > 1 ? rpy[1] : 0.0;
    const double yaw = rpy.size() > 2 ? rpy[2] : 0.0;
    R_mount_ = rotation_from_rpy(roll, pitch, yaw);
    R_mount_inv_ = transpose(R_mount_);
    cloud_to_base_ = mount_rotation_mode_ == "direct" ? R_mount_ : R_mount_inv_;
  }

  void imu_callback(const Imu::SharedPtr msg)
  {
    imu_history_.push_back(*msg);
    while (imu_history_.size() > 400) {
      imu_history_.pop_front();
    }
  }

  // Complementary filter tracking the gravity-up direction in the (head) IMU
  // frame: gyro propagates it (a world-fixed vector's IMU coords evolve as
  // d(up)/dt = -w x up), the accelerometer slowly corrects gyro drift. The small
  // gain rejects gait acceleration; the gyro carries the fast tilt excursions.
  void leveling_imu_callback(const Imu::SharedPtr msg)
  {
    const double t = stamp_to_sec(msg->header.stamp);
    const Vec3 a_raw{
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    const double a_mag = std::sqrt(dot(a_raw, a_raw));
    const Vec3 a = normalize(a_raw);
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z)) {
      return;
    }
    if (!bias_estimated_) {
      if (bias_init_start_ < 0.0) {
        bias_init_start_ = t;
      }
      bias_accum_ = add(
        bias_accum_,
        {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z});
      ++bias_count_;
      if (t - bias_init_start_ >= head_imu_bias_init_sec_ && bias_count_ > 0) {
        head_imu_gyro_bias_ = {
          bias_accum_.x / bias_count_, bias_accum_.y / bias_count_, bias_accum_.z / bias_count_};
        bias_estimated_ = true;
        RCLCPP_INFO(
          get_logger(), "head_imu_leveling: auto gyro bias [%.4f, %.4f, %.4f] rad/s (%zu samples)",
          head_imu_gyro_bias_.x, head_imu_gyro_bias_.y, head_imu_gyro_bias_.z, bias_count_);
      }
    }
    if (!have_grav_) {
      grav_up_ = a;
      have_grav_ = true;
      last_leveling_imu_time_ = t;
      return;
    }
    const double dt = t - last_leveling_imu_time_;
    last_leveling_imu_time_ = t;
    if (dt > 0.0 && dt < head_imu_max_dt_) {
      const Vec3 w{
        msg->angular_velocity.x - head_imu_gyro_bias_.x,
        msg->angular_velocity.y - head_imu_gyro_bias_.y,
        msg->angular_velocity.z - head_imu_gyro_bias_.z};
      const Vec3 dgrav = cross(w, grav_up_);
      grav_up_ = normalize(
        {grav_up_.x - dgrav.x * dt, grav_up_.y - dgrav.y * dt, grav_up_.z - dgrav.z * dt});
      // Online bias adaptation: only the gravity-perpendicular bias is observable
      // from the accelerometer (it is the leveling-relevant part), and only on
      // near-gravity samples so gait acceleration does not poison it. e = up x a
      // is the residual the fixed bias leaves; integrating it tracks the drift.
      if (head_imu_bias_learn_rate_ > 0.0 &&
        std::abs(a_mag - kGravity) < head_imu_bias_accel_tol_)
      {
        const Vec3 e = cross(grav_up_, a);
        head_imu_gyro_bias_.x -= head_imu_bias_learn_rate_ * e.x * dt;
        head_imu_gyro_bias_.y -= head_imu_bias_learn_rate_ * e.y * dt;
        head_imu_gyro_bias_.z -= head_imu_bias_learn_rate_ * e.z * dt;
      }
    }
    const double k = head_imu_comp_gain_;
    grav_up_ = normalize(
      {grav_up_.x + k * (a.x - grav_up_.x),
        grav_up_.y + k * (a.y - grav_up_.y),
        grav_up_.z + k * (a.z - grav_up_.z)});
  }

  void odom_callback(const Odometry::SharedPtr msg)
  {
    odom_history_.push_back(*msg);
    while (odom_history_.size() > 200) {
      odom_history_.pop_front();
    }
  }

  template<typename MsgT>
  std::pair<bool, double> nearest_msg(
    const std::deque<MsgT> & history,
    double stamp_sec,
    double timeout_sec,
    MsgT & out) const
  {
    if (history.empty()) {
      return {false, std::numeric_limits<double>::quiet_NaN()};
    }
    auto best = history.begin();
    double best_dt = std::abs(stamp_to_sec(best->header.stamp) - stamp_sec);
    for (auto it = std::next(history.begin()); it != history.end(); ++it) {
      const double dt = std::abs(stamp_to_sec(it->header.stamp) - stamp_sec);
      if (dt < best_dt) {
        best = it;
        best_dt = dt;
      }
    }
    if (best_dt > timeout_sec) {
      return {false, best_dt};
    }
    out = *best;
    return {true, best_dt};
  }

  struct StateSelection
  {
    Vec3 linear_sensor;
    Vec3 angular_sensor;
    bool has_odom{false};
    bool has_imu{false};
    Odometry odom;
    Imu imu;
    double odom_age{std::numeric_limits<double>::quiet_NaN()};
    double imu_age{std::numeric_limits<double>::quiet_NaN()};
  };

  StateSelection select_state(double stamp_sec) const
  {
    StateSelection state;
    if (use_twist_) {
      auto result = nearest_msg(odom_history_, stamp_sec, odom_timeout_sec_, state.odom);
      state.has_odom = result.first;
      state.odom_age = result.second;
    }
    if (use_imu_) {
      auto result = nearest_msg(imu_history_, stamp_sec, imu_timeout_sec_, state.imu);
      state.has_imu = result.first;
      state.imu_age = result.second;
    }

    Vec3 linear_velocity;
    Vec3 angular_velocity;
    if (state.has_odom && use_twist_) {
      const auto & twist = state.odom.twist.twist;
      linear_velocity = {twist.linear.x, twist.linear.y, twist.linear.z};
      angular_velocity = {twist.angular.x, twist.angular.y, twist.angular.z};
    }
    if (state.has_imu && use_imu_) {
      angular_velocity = {
        state.imu.angular_velocity.x,
        state.imu.angular_velocity.y,
        state.imu.angular_velocity.z};
    }
    if (!use_3d_) {
      linear_velocity.z = 0.0;
      angular_velocity.x = 0.0;
      angular_velocity.y = 0.0;
    }

    const Vec3 r_sensor{sensor_x_in_base_, sensor_y_in_base_, sensor_z_in_base_};
    state.linear_sensor = R_mount_ * add(linear_velocity, cross(angular_velocity, r_sensor));
    state.angular_sensor = R_mount_ * angular_velocity;
    return state;
  }

  std::pair<bool, FieldInfo> resolve_time_field(const PointCloud2 & msg) const
  {
    if (!timestamp_field_name_.empty()) {
      const auto field = find_field(msg, timestamp_field_name_);
      if (field.found) {
        return {true, field};
      }
    }
    for (const auto & candidate : timestamp_field_candidates_) {
      const auto field = find_field(msg, candidate);
      if (field.found) {
        return {true, field};
      }
    }
    return {false, {}};
  }

  std::vector<double> relative_times_from_field(const std::vector<double> & values) const
  {
    std::vector<double> result(values.size(), 0.0);
    bool any = false;
    double min_value = std::numeric_limits<double>::infinity();
    double max_abs = 0.0;
    for (const double value : values) {
      if (!std::isfinite(value)) {
        continue;
      }
      any = true;
      min_value = std::min(min_value, value);
      max_abs = std::max(max_abs, std::abs(value));
    }
    if (!any) {
      return result;
    }

    double scale = 1.0;
    if (timestamp_unit_ == "seconds") {
      scale = 1.0;
    } else if (timestamp_unit_ == "milliseconds") {
      scale = 1e-3;
    } else if (timestamp_unit_ == "microseconds") {
      scale = 1e-6;
    } else if (timestamp_unit_ == "nanoseconds") {
      scale = 1e-9;
    } else if (max_abs <= 1.0) {
      scale = 1.0;
    } else if (max_abs <= 1e3) {
      scale = 1e-3;
    } else if (max_abs <= 1e6) {
      scale = 1e-6;
    } else {
      scale = 1e-9;
    }

    for (size_t i = 0; i < values.size(); ++i) {
      if (std::isfinite(values[i])) {
        result[i] = std::max(0.0, (values[i] - min_value) * scale);
      }
    }
    return result;
  }

  std::vector<double> relative_times_from_azimuth(
    const std::vector<double> & x,
    const std::vector<double> & y) const
  {
    std::vector<double> times(x.size(), 0.0);
    size_t first_finite = 0;
    bool found = false;
    std::vector<double> azimuths(x.size(), 0.0);
    for (size_t i = 0; i < x.size(); ++i) {
      if (!std::isfinite(x[i]) || !std::isfinite(y[i])) {
        continue;
      }
      double azimuth = std::atan2(y[i], x[i]);
      if (azimuth < 0.0) {
        azimuth += 2.0 * kPi;
      }
      azimuths[i] = azimuth;
      if (!found) {
        first_finite = i;
        found = true;
      }
    }
    if (!found) {
      return times;
    }

    const double start = azimuths[first_finite];
    for (size_t i = 0; i < x.size(); ++i) {
      if (!std::isfinite(x[i]) || !std::isfinite(y[i])) {
        continue;
      }
      const double progress = azimuth_direction_ == "cw" ?
        std::fmod(start - azimuths[i] + 2.0 * kPi, 2.0 * kPi) :
        std::fmod(azimuths[i] - start + 2.0 * kPi, 2.0 * kPi);
      times[i] = (progress / (2.0 * kPi)) * scan_period_;
    }
    return times;
  }

  std::vector<double> relative_times_from_index(size_t n) const
  {
    std::vector<double> times(n, 0.0);
    if (n <= 1) {
      return times;
    }
    for (size_t i = 0; i < n; ++i) {
      times[i] = (static_cast<double>(i) / static_cast<double>(n - 1)) * scan_period_;
    }
    return times;
  }

  std::vector<double> relative_times(
    const PointCloud2 & msg,
    const std::vector<double> & x,
    const std::vector<double> & y,
    size_t n_points)
  {
    const auto time_field = resolve_time_field(msg);
    if (time_field.first) {
      std::vector<double> values(n_points, 0.0);
      for (size_t i = 0; i < n_points; ++i) {
        values[i] = read_field_value(msg.data.data() + i * msg.point_step, time_field.second);
      }
      last_timing_mode_ = "field:" + time_field.second.name;
      return relative_times_from_field(values);
    }
    if (fallback_mode_ == "index") {
      last_timing_mode_ = "fallback:index";
      return relative_times_from_index(n_points);
    }
    if (fallback_mode_ == "azimuth") {
      last_timing_mode_ = "fallback:azimuth";
      return relative_times_from_azimuth(x, y);
    }
    // fallback_mode == "disabled" (or unknown): zero rel times, caller must
    // detect via has_time_field() and apply missing-state drop policy.
    last_timing_mode_ = "fallback:disabled";
    return std::vector<double>(n_points, 0.0);
  }

  bool has_time_field(const PointCloud2 & msg) const
  {
    return resolve_time_field(msg).first;
  }

  void publish_diagnostics(
    int8_t level,
    const std::string & message,
    size_t n_points,
    double cloud_stamp,
    double imu_age_sec,
    double odom_age_sec,
    double processing_time_ms)
  {
    if (!publish_diagnostics_ || !diagnostics_pub_) {
      return;
    }

    DiagnosticStatus status;
    status.name = "distortion_corrector";
    status.hardware_id = "lidar";
    status.level = level;
    status.message = message;
    status.values = {
      kv("input_topic", input_topic_),
      kv("output_topic", output_topic_),
      kv("point_count", std::to_string(n_points)),
      kv("cloud_stamp_sec", format_double(cloud_stamp, 6)),
      kv("nearest_imu_age_sec", format_double(imu_age_sec, 6)),
      kv("nearest_odom_age_sec", format_double(odom_age_sec, 6)),
      kv("processing_time_ms", format_double(processing_time_ms, 3)),
      kv("timing_mode", last_timing_mode_),
      kv("deskew_enabled", deskew_enabled_ ? "true" : "false"),
      kv("last_deskew_applied", last_deskew_applied_ ? "true" : "false"),
      kv("last_gravity_aligned", last_gravity_aligned_ ? "true" : "false"),
      kv("last_self_filter_count", std::to_string(last_self_filter_count_)),
      kv("processed_count", std::to_string(processed_count_)),
      kv("passthrough_count", std::to_string(passthrough_count_)),
    };

    DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  double elapsed_ms(const std::chrono::steady_clock::time_point & start_time) const
  {
    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    return std::chrono::duration<double, std::milli>(elapsed).count();
  }

  void publish_passthrough(
    const PointCloud2::SharedPtr msg,
    const std::string & reason,
    const std::chrono::steady_clock::time_point & start_time)
  {
    ++passthrough_count_;
    publisher_->publish(*msg);
    publish_diagnostics(
      DiagnosticStatus::WARN, reason,
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height),
      stamp_to_sec(msg->header.stamp), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), elapsed_ms(start_time));
  }

  // Missing-state drop: emit diagnostic but do not publish a cloud.
  void drop_frame(
    const PointCloud2::SharedPtr msg,
    const std::string & reason,
    const std::chrono::steady_clock::time_point & start_time)
  {
    ++passthrough_count_;
    publish_diagnostics(
      DiagnosticStatus::ERROR, reason,
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height),
      stamp_to_sec(msg->header.stamp), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), elapsed_ms(start_time));
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "distortion_corrector: %s", reason.c_str());
  }

  // Fit the ground-plane normal (unit, pointing up) from the deskewed cloud by
  // least squares (z = a*x + b*y + c) over floor-band candidates, with two
  // outlier-rejection passes. Returns {false, _} when too few candidates.
  std::pair<bool, Vec3> fit_floor_normal(
    const std::vector<float> & cx, const std::vector<float> & cy,
    const std::vector<float> & cz, const std::vector<bool> & finite, size_t n_points) const
  {
    std::vector<std::array<double, 3>> pts;
    pts.reserve(4096);
    for (size_t i = 0; i < n_points; i += 3) {  // subsample for speed
      if (!finite[i]) {
        continue;
      }
      const double xv = cx[i];
      const double yv = cy[i];
      const double zv = cz[i];
      const double r = std::hypot(xv, yv);
      if (zv > floor_z_min_ && zv < floor_z_max_ && r > floor_r_min_ && r < floor_r_max_) {
        pts.push_back({xv, yv, zv});
      }
    }
    if (pts.size() < floor_min_points_) {
      return {false, Vec3{}};
    }
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    for (int iter = 0; iter < 3; ++iter) {
      double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, s1 = 0, sxz = 0, syz = 0, sz = 0;
      size_t used = 0;
      for (const auto & p : pts) {
        if (iter > 0 && std::abs(a * p[0] + b * p[1] + c - p[2]) > floor_inlier_thresh_) {
          continue;
        }
        sxx += p[0] * p[0]; sxy += p[0] * p[1]; sx += p[0];
        syy += p[1] * p[1]; sy += p[1]; s1 += 1.0;
        sxz += p[0] * p[2]; syz += p[1] * p[2]; sz += p[2];
        ++used;
      }
      if (used < floor_min_points_ / 2) {
        break;
      }
      // Solve the 3x3 normal equations by Cramer's rule.
      const double det =
        sxx * (syy * s1 - sy * sy) - sxy * (sxy * s1 - sy * sx) + sx * (sxy * sy - syy * sx);
      if (std::abs(det) < 1e-9) {
        break;
      }
      a = (sxz * (syy * s1 - sy * sy) - sxy * (syz * s1 - sy * sz) + sx * (syz * sy - syy * sz)) / det;
      b = (sxx * (syz * s1 - sy * sz) - sxz * (sxy * s1 - sy * sx) + sx * (sxy * sz - syz * sx)) / det;
      c = (sxx * (syy * sz - syz * sy) - sxy * (sxy * sz - syz * sx) + sxz * (sxy * sy - syy * sx)) / det;
    }
    Vec3 n = normalize({-a, -b, 1.0});
    if (n.z < 0.0) {
      n = {-n.x, -n.y, -n.z};
    }
    return {true, n};
  }

  void cloud_callback(const PointCloud2::SharedPtr msg)
  {
    const auto start_time = std::chrono::steady_clock::now();
    const size_t n_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const double cloud_stamp = stamp_to_sec(msg->header.stamp);
    last_self_filter_count_ = 0;
    last_deskew_applied_ = false;
    last_gravity_aligned_ = false;
    last_timing_mode_ = deskew_enabled_ ? "unresolved" : "deskew:disabled";

    if (n_points == 0) {
      ++processed_count_;
      publisher_->publish(*msg);
      publish_diagnostics(
        DiagnosticStatus::OK, "empty cloud", n_points, cloud_stamp,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        elapsed_ms(start_time));
      return;
    }

    const auto x_field = find_field(*msg, "x");
    const auto y_field = find_field(*msg, "y");
    const auto z_field = find_field(*msg, "z");
    if (!x_field.found || !y_field.found || !z_field.found) {
      publish_passthrough(msg, "cloud missing x/y/z fields", start_time);
      return;
    }
    if (!is_float_field(x_field) || !is_float_field(y_field) || !is_float_field(z_field)) {
      publish_passthrough(msg, "cloud x/y/z fields are not floating point", start_time);
      return;
    }

    if (deskew_enabled_ && fallback_mode_ == "disabled" && !has_time_field(*msg)) {
      drop_frame(msg, "deskew enabled with fallback_mode=disabled but cloud has no time field", start_time);
      return;
    }

    StateSelection state = select_state(cloud_stamp);
    if (state.has_imu && imu_stale_warn_sec_ > 0.0) {
      // nearest_msg() always finds a buffered sample within ~10 ms of any
      // stamp inside the buffered past, so imu_timeout_sec cannot catch
      // stale cloud stamps; only an age check against now() can.
      const double now_sec = stamp_to_sec(get_clock()->now());
      const double sample_age = now_sec - stamp_to_sec(state.imu.header.stamp);
      if (now_sec > 0.0 && sample_age > imu_stale_warn_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "distortion_corrector: matched IMU sample is %.2f s older than now(); "
          "cloud stamps are likely stale (deskew/leveling would use old attitude)",
          sample_age);
      }
    }
    const bool can_deskew =
      deskew_enabled_ &&
      (!use_twist_ || state.has_odom) &&
      (!use_imu_ || state.has_imu);

    std::vector<double> x(n_points);
    std::vector<double> y(n_points);
    std::vector<double> z(n_points);
    std::vector<bool> finite(n_points, false);
    for (size_t i = 0; i < n_points; ++i) {
      const uint8_t * point = msg->data.data() + i * msg->point_step;
      x[i] = read_field_value(point, x_field);
      y[i] = read_field_value(point, y_field);
      z[i] = read_field_value(point, z_field);
      finite[i] = std::isfinite(x[i]) && std::isfinite(y[i]) && std::isfinite(z[i]);
    }

    std::vector<float> cx(n_points);
    std::vector<float> cy(n_points);
    std::vector<float> cz(n_points);
    double deskew_ref_offset = 0.0;
    if (can_deskew) {
      const auto rel_t = relative_times(*msg, x, y, n_points);
      // Reference instant the deskew unwinds to (and that the output stamp is
      // shifted to). "start" (rel_t=0, earliest point) is the historical
      // behavior; "median"/"mean" match DLIO's deskewed cloud, which references
      // the sweep's MEDIAN point time -- so both clouds represent the same instant
      // and stop disagreeing in yaw during turns (~half-sweep offset otherwise).
      if (deskew_reference_ == "median" || deskew_reference_ == "mean") {
        std::vector<double> valid;
        valid.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i) {
          if (finite[i] && std::isfinite(rel_t[i])) {
            valid.push_back(rel_t[i]);
          }
        }
        if (!valid.empty()) {
          if (deskew_reference_ == "mean") {
            double s = 0.0;
            for (const double v : valid) {
              s += v;
            }
            deskew_ref_offset = s / static_cast<double>(valid.size());
          } else {
            const size_t mid = valid.size() / 2;
            std::nth_element(valid.begin(), valid.begin() + mid, valid.end());
            deskew_ref_offset = valid[mid];
          }
        }
      }
      const double wx = state.angular_sensor.x;
      const double wy = state.angular_sensor.y;
      const double wz = state.angular_sensor.z;
      const double vx = state.linear_sensor.x;
      const double vy = state.linear_sensor.y;
      const double vz = state.linear_sensor.z;
      for (size_t i = 0; i < n_points; ++i) {
        if (finite[i]) {
          const double dt = rel_t[i] - deskew_ref_offset;
          cx[i] = static_cast<float>(x[i] + dt * (vx + wy * z[i] - wz * y[i]));
          cy[i] = static_cast<float>(y[i] + dt * (vy + wz * x[i] - wx * z[i]));
          cz[i] = static_cast<float>(z[i] + dt * (vz + wx * y[i] - wy * x[i]));
        } else {
          cx[i] = static_cast<float>(x[i]);
          cy[i] = static_cast<float>(y[i]);
          cz[i] = static_cast<float>(z[i]);
        }
      }
      last_deskew_applied_ = true;
    } else {
      if (deskew_enabled_) {
        last_timing_mode_ = "deskew:missing_state";
      }
      for (size_t i = 0; i < n_points; ++i) {
        cx[i] = static_cast<float>(x[i]);
        cy[i] = static_cast<float>(y[i]);
        cz[i] = static_cast<float>(z[i]);
      }
    }

    const float nan32 = std::numeric_limits<float>::quiet_NaN();
    if (filter_self_) {
      const double min_radius_sq = min_radius_ * min_radius_;
      for (size_t i = 0; i < n_points; ++i) {
        if (!finite[i]) {
          continue;
        }
        const Vec3 p_cloud{cx[i], cy[i], cz[i]};
        const Vec3 p_base = cloud_to_base_ * p_cloud;
        const double x_b = p_base.x + sensor_x_in_base_;
        const double y_b = p_base.y + sensor_y_in_base_;
        const double z_b = p_base.z + sensor_z_in_base_;
        const double radius_sq =
          static_cast<double>(cx[i]) * cx[i] + static_cast<double>(cy[i]) * cy[i] +
          static_cast<double>(cz[i]) * cz[i];
        const bool inside = radius_sq < min_radius_sq ||
          ((x_b >= robot_min_x_ && x_b <= robot_max_x_) &&
          (y_b >= robot_min_y_ && y_b <= robot_max_y_) &&
          (z_b >= robot_min_z_ && z_b <= robot_max_z_));
        if (inside) {
          ++last_self_filter_count_;
          cx[i] = nan32;
          cy[i] = nan32;
          cz[i] = nan32;
        }
      }
    }

    bool gravity_aligned = false;
    Mat3 R_level;
    bool have_R_level = false;
    // Priority: floor-plane leveling (ground truth, no IMU) -> head-IMU leveling
    // (tracks tilt where the floor is not visible) -> static mount righting below.
    if (gravity_align_ && floor_plane_leveling_) {
      const auto fit = fit_floor_normal(cx, cy, cz, finite, n_points);
      if (fit.first && fit.second.z > floor_max_tilt_cos_) {
        if (!have_floor_normal_) {
          floor_normal_filt_ = fit.second;
          have_floor_normal_ = true;
        } else {
          const double al = floor_normal_lowpass_;
          floor_normal_filt_ = normalize(
            {floor_normal_filt_.x + al * (fit.second.x - floor_normal_filt_.x),
              floor_normal_filt_.y + al * (fit.second.y - floor_normal_filt_.y),
              floor_normal_filt_.z + al * (fit.second.z - floor_normal_filt_.z)});
        }
      }
      if (have_floor_normal_) {  // hold the last good normal when a sweep has no floor
        R_level = rotation_between(floor_normal_filt_, Vec3{0.0, 0.0, 1.0});
        have_R_level = true;
      }
    }
    if (!have_R_level && gravity_align_ && head_imu_leveling_ && have_grav_) {
      // Estimated cloud floor-normal = extrinsic * complementary-filtered gravity-up.
      const Vec3 n_up = normalize(R_head_imu_to_cloud_ * grav_up_);
      R_level = rotation_between(n_up, Vec3{0.0, 0.0, 1.0});
      have_R_level = true;
    }
    if (have_R_level) {
      for (size_t i = 0; i < n_points; ++i) {
        if (!std::isfinite(cx[i]) || !std::isfinite(cy[i]) || !std::isfinite(cz[i])) {
          continue;
        }
        const Vec3 p_stab = R_level * Vec3{cx[i], cy[i], cz[i]};
        cx[i] = static_cast<float>(p_stab.x);
        cy[i] = static_cast<float>(p_stab.y);
        cz[i] = static_cast<float>(p_stab.z);
      }
      gravity_aligned = true;
      last_gravity_aligned_ = true;
    } else if (gravity_align_) {
      // Attenuation 0 needs no IMU sample (pure mount righting); any IMU
      // contribution keeps the original skip-on-missing-IMU fallback.
      const bool imu_needed = gravity_align_attenuation_ > 0.0;
      const bool imu_ready = use_imu_ && state.has_imu;
      if (!imu_needed || imu_ready) {
        double roll = 0.0;
        double pitch = 0.0;
        if (imu_needed) {
          const auto rp = roll_pitch_from_imu(state.imu.orientation);
          roll = gravity_align_attenuation_ * rp.first;
          pitch = gravity_align_attenuation_ * rp.second;
        }
        const Mat3 R_body = rotation_from_rpy(roll, pitch, 0.0);
        for (size_t i = 0; i < n_points; ++i) {
          if (!std::isfinite(cx[i]) || !std::isfinite(cy[i]) || !std::isfinite(cz[i])) {
            continue;
          }
          const Vec3 p_cloud{cx[i], cy[i], cz[i]};
          const Vec3 p_stab = R_body * (cloud_to_base_ * p_cloud);
          cx[i] = static_cast<float>(p_stab.x);
          cy[i] = static_cast<float>(p_stab.y);
          cz[i] = static_cast<float>(p_stab.z);
        }
        gravity_aligned = true;
        last_gravity_aligned_ = true;
      }
    }

    PointCloud2 out_msg = *msg;
    if (output_stamp_mode_ == "now") {
      out_msg.header.stamp = get_clock()->now();
    } else if (deskew_ref_offset != 0.0) {
      // Stamp the cloud at the deskew reference instant so the TF lookup that
      // positions this body-frame cloud uses the matching pose (no yaw lag).
      const double shifted = stamp_to_sec(msg->header.stamp) + deskew_ref_offset;
      const double whole = std::floor(shifted);
      out_msg.header.stamp.sec = static_cast<int32_t>(whole);
      out_msg.header.stamp.nanosec = static_cast<uint32_t>((shifted - whole) * 1e9);
    }
    if (gravity_aligned) {
      out_msg.header.frame_id = gravity_align_frame_;
    }
    out_msg.data = msg->data;
    for (size_t i = 0; i < n_points; ++i) {
      uint8_t * point = out_msg.data.data() + i * out_msg.point_step;
      write_float_field(point, x_field, cx[i]);
      write_float_field(point, y_field, cy[i]);
      write_float_field(point, z_field, cz[i]);
    }
    out_msg.is_dense = msg->is_dense && !filter_self_;

    publisher_->publish(out_msg);
    ++processed_count_;
    publish_diagnostics(
      DiagnosticStatus::OK, "processed cloud published", n_points, cloud_stamp,
      state.imu_age, state.odom_age, elapsed_ms(start_time));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  bool deskew_enabled_{false};
  bool use_imu_{true};
  bool use_twist_{true};
  bool use_3d_{true};
  double scan_period_{0.1};
  std::string mount_rotation_mode_{"inverse"};
  std::string timestamp_field_name_;
  std::vector<std::string> timestamp_field_candidates_;
  std::string timestamp_unit_{"auto"};
  std::string fallback_mode_{"azimuth"};
  std::string azimuth_direction_{"cw"};
  std::string deskew_reference_{"start"};
  double imu_timeout_sec_{0.25};
  double odom_timeout_sec_{0.25};
  std::string output_stamp_mode_{"input"};
  bool gravity_align_{true};
  std::string gravity_align_frame_{"lidar_stabilized"};
  double gravity_align_attenuation_{1.0};
  bool head_imu_leveling_{false};
  bool floor_plane_leveling_{false};
  double floor_z_min_{-1.6};
  double floor_z_max_{-0.7};
  double floor_r_min_{0.5};
  double floor_r_max_{6.0};
  size_t floor_min_points_{250};
  double floor_inlier_thresh_{0.05};
  double floor_normal_lowpass_{0.2};
  double floor_max_tilt_cos_{0.978};
  Vec3 floor_normal_filt_{0.0, 0.0, 1.0};
  bool have_floor_normal_{false};
  std::string leveling_imu_topic_;
  double head_imu_comp_gain_{0.01};
  double head_imu_max_dt_{0.1};
  Mat3 R_head_imu_to_cloud_;
  Vec3 head_imu_gyro_bias_{0.0, 0.0, 0.0};
  double head_imu_bias_init_sec_{2.0};
  double head_imu_bias_learn_rate_{0.02};
  double head_imu_bias_accel_tol_{1.0};
  bool bias_estimated_{false};
  Vec3 bias_accum_{0.0, 0.0, 0.0};
  size_t bias_count_{0};
  double bias_init_start_{-1.0};
  Vec3 grav_up_{0.0, 0.0, -1.0};
  bool have_grav_{false};
  double last_leveling_imu_time_{-1.0};
  double imu_stale_warn_sec_{0.5};
  bool filter_self_{false};
  double robot_min_x_{-0.40};
  double robot_max_x_{0.36};
  double robot_min_y_{-0.20};
  double robot_max_y_{0.20};
  double robot_min_z_{-0.25};
  double robot_max_z_{0.10};
  double min_radius_{0.12};
  double sensor_x_in_base_{0.28945};
  double sensor_y_in_base_{0.0};
  double sensor_z_in_base_{-0.046825};
  bool publish_diagnostics_{false};
  std::string diagnostics_topic_;
  Mat3 R_mount_;
  Mat3 R_mount_inv_;
  Mat3 cloud_to_base_;

  std::deque<Imu> imu_history_;
  std::deque<Odometry> odom_history_;
  std::string last_timing_mode_{"unresolved"};
  size_t processed_count_{0};
  size_t passthrough_count_{0};
  size_t last_self_filter_count_{0};
  bool last_deskew_applied_{false};
  bool last_gravity_aligned_{false};

  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Imu>::SharedPtr leveling_imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistortionCorrector>());
  rclcpp::shutdown();
  return 0;
}
