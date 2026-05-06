#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

std::pair<double, double> roll_pitch_from_imu(const geometry_msgs::msg::Quaternion & q)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));
  return {roll, pitch};
}

std::array<double, 9> rotation_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  return {
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  };
}

std::array<double, 3> multiply(const std::array<double, 9> & m, const std::array<double, 3> & v)
{
  return {
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  };
}

}  // namespace

class LidarLevelFramePublisher : public rclcpp::Node
{
public:
  LidarLevelFramePublisher()
  : Node("lidar_level_frame_publisher")
  {
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("parent_frame", "base_footprint");
    declare_parameter<std::string>("child_frame", "lidar_stabilized");
    declare_parameter<double>("lidar_x_in_base", 0.28945);
    declare_parameter<double>("lidar_y_in_base", 0.0);
    declare_parameter<double>("lidar_z_in_base", -0.046825);
    declare_parameter<double>("standing_height", 0.28);

    imu_topic_ = get_parameter("imu_topic").as_string();
    parent_frame_ = get_parameter("parent_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();
    lidar_in_base_ = {
      get_parameter("lidar_x_in_base").as_double(),
      get_parameter("lidar_y_in_base").as_double(),
      get_parameter("lidar_z_in_base").as_double(),
    };
    standing_height_ = get_parameter("standing_height").as_double();

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLevelFramePublisher::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "lidar_level_frame_publisher: %s -> %s from %s "
      "(lidar_in_base=[%.5f, %.5f, %.5f], standing_height=%.3f)",
      parent_frame_.c_str(), child_frame_.c_str(), imu_topic_.c_str(),
      lidar_in_base_[0], lidar_in_base_[1], lidar_in_base_[2], standing_height_);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto [roll, pitch] = roll_pitch_from_imu(msg->orientation);
    const auto R_body = rotation_from_rpy(roll, pitch, 0.0);
    const auto leveled_position = multiply(R_body, lidar_in_base_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = leveled_position[0];
    tf_msg.transform.translation.y = leveled_position[1];
    tf_msg.transform.translation.z = leveled_position[2] + standing_height_;

    // Keep the LiDAR processing frame gravity-level: position follows trunk motion,
    // orientation does not inherit trunk roll or pitch.
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    broadcaster_->sendTransform(tf_msg);
  }

  std::string imu_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  std::array<double, 3> lidar_in_base_{};
  double standing_height_{0.28};

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLevelFramePublisher>());
  rclcpp::shutdown();
  return 0;
}
