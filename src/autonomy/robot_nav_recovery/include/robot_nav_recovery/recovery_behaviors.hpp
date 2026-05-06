#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace robot_nav_recovery
{

class GuardedSpin : public nav2_behaviors::TimedBehavior<nav2_msgs::action::Spin>
{
public:
  GuardedSpin();

  void onConfigure() override;
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const nav2_msgs::action::Spin::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

private:
  bool isCollisionFree(double remaining_yaw, const geometry_msgs::msg::Twist & cmd_vel);

  nav2_msgs::action::Spin::Feedback::SharedPtr feedback_;
  double min_rotational_vel_{0.02};
  double max_rotational_vel_{0.4};
  double rotational_acc_lim_{0.6};
  double simulate_ahead_time_{1.0};
  double cmd_yaw_{0.0};
  double prev_yaw_{0.0};
  double relative_yaw_{0.0};
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

class GuardedBackUp : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
  GuardedBackUp();

  void onConfigure() override;
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const nav2_msgs::action::BackUp::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;
  void onActionCompletion() override;

private:
  bool isCollisionFree(double traveled, const geometry_msgs::msg::Twist & cmd_vel);
  double nextVelocity(double remaining_distance);

  nav2_msgs::action::BackUp::Feedback::SharedPtr feedback_;
  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_distance_{0.0};
  double command_speed_{0.0};
  double simulate_ahead_time_{1.0};
  double acceleration_limit_{0.0};
  double deceleration_limit_{0.0};
  double minimum_speed_{0.0};
  double last_vel_{0.0};
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

class GuardedLateralStep
  : public nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>
{
public:
  GuardedLateralStep();

  void onConfigure() override;
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const nav2_msgs::action::DriveOnHeading::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;
  void onActionCompletion() override;

private:
  bool isCollisionFree(double traveled, const geometry_msgs::msg::Twist & cmd_vel);
  double nextVelocity(double remaining_distance);

  nav2_msgs::action::DriveOnHeading::Feedback::SharedPtr feedback_;
  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_distance_{0.0};
  double command_speed_{0.0};
  double simulate_ahead_time_{1.0};
  double acceleration_limit_{0.0};
  double deceleration_limit_{0.0};
  double minimum_speed_{0.0};
  double last_vel_{0.0};
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace robot_nav_recovery
