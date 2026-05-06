#include "robot_nav_recovery/recovery_behaviors.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace robot_nav_recovery
{
namespace
{

double distanceFromStart(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & current)
{
  return std::hypot(
    current.pose.position.x - start.pose.position.x,
    current.pose.position.y - start.pose.position.y);
}

void declareMotionLimits(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & behavior_name,
  double & acceleration_limit,
  double & deceleration_limit,
  double & minimum_speed)
{
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name + ".acceleration_limit", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name + ".deceleration_limit", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name + ".minimum_speed", rclcpp::ParameterValue(0.0));

  node->get_parameter(behavior_name + ".acceleration_limit", acceleration_limit);
  node->get_parameter(behavior_name + ".deceleration_limit", deceleration_limit);
  node->get_parameter(behavior_name + ".minimum_speed", minimum_speed);
  acceleration_limit = std::fabs(acceleration_limit);
  deceleration_limit = -std::fabs(deceleration_limit);
  minimum_speed = std::fabs(minimum_speed);
}

double limitedVelocity(
  double requested_speed,
  double last_vel,
  double remaining_distance,
  double acceleration_limit,
  double deceleration_limit,
  double minimum_speed,
  double cycle_frequency)
{
  if (acceleration_limit == 0.0 || deceleration_limit == 0.0) {
    return requested_speed;
  }

  const bool positive = requested_speed > 0.0;
  const double min_feasible =
    positive ? last_vel + deceleration_limit / cycle_frequency :
    last_vel - acceleration_limit / cycle_frequency;
  const double max_feasible =
    positive ? last_vel + acceleration_limit / cycle_frequency :
    last_vel - deceleration_limit / cycle_frequency;
  double cmd = std::clamp(requested_speed, min_feasible, max_feasible);

  const double max_stop_speed = std::sqrt(std::max(0.0, -2.0 * deceleration_limit * remaining_distance));
  if (std::fabs(cmd) > max_stop_speed) {
    cmd = positive ? max_stop_speed : -max_stop_speed;
  }
  if (std::fabs(cmd) < minimum_speed) {
    cmd = positive ? minimum_speed : -minimum_speed;
  }
  return cmd;
}

bool simulateCollisionFree(
  const std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> & checker,
  geometry_msgs::msg::Pose2D pose,
  const geometry_msgs::msg::Twist & cmd_vel,
  double cycle_frequency,
  double simulate_ahead_time,
  double remaining_distance,
  bool distance_limited)
{
  bool fetch_data = true;
  const auto initial_pose = pose;
  const int max_cycle_count = static_cast<int>(cycle_frequency * simulate_ahead_time);
  for (int cycle = 0; cycle < max_cycle_count; ++cycle) {
    const double dt = static_cast<double>(cycle) / cycle_frequency;
    pose.theta = initial_pose.theta + cmd_vel.angular.z * dt;
    pose.x = initial_pose.x +
      cmd_vel.linear.x * dt * std::cos(initial_pose.theta) -
      cmd_vel.linear.y * dt * std::sin(initial_pose.theta);
    pose.y = initial_pose.y +
      cmd_vel.linear.x * dt * std::sin(initial_pose.theta) +
      cmd_vel.linear.y * dt * std::cos(initial_pose.theta);

    if (distance_limited) {
      const double traveled = std::hypot(pose.x - initial_pose.x, pose.y - initial_pose.y);
      if (traveled >= remaining_distance) {
        break;
      }
    }

    if (!checker->isCollisionFree(pose, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace

GuardedSpin::GuardedSpin()
: nav2_behaviors::TimedBehavior<nav2_msgs::action::Spin>(),
  feedback_(std::make_shared<nav2_msgs::action::Spin::Feedback>())
{
}

void GuardedSpin::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "max_rotational_vel", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, "min_rotational_vel", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, "rotational_acc_lim", rclcpp::ParameterValue(0.6));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  node->get_parameter("max_rotational_vel", max_rotational_vel_);
  node->get_parameter("min_rotational_vel", min_rotational_vel_);
  node->get_parameter("rotational_acc_lim", rotational_acc_lim_);
}

nav2_behaviors::Status GuardedSpin::onRun(
  const std::shared_ptr<const nav2_msgs::action::Spin::Goal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Guarded spin cannot get the robot pose.");
    return nav2_behaviors::Status::FAILED;
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
  relative_yaw_ = 0.0;
  cmd_yaw_ = command->target_yaw;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status GuardedSpin::onCycleUpdate()
{
  if ((end_time_ - clock_->now()).seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Guarded spin lost the robot pose.");
    return nav2_behaviors::Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double delta_yaw = current_yaw - prev_yaw_;
  if (std::fabs(delta_yaw) > M_PI) {
    delta_yaw = std::copysign(2.0 * M_PI - std::fabs(delta_yaw), prev_yaw_);
  }
  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;
  feedback_->angular_distance_traveled = static_cast<float>(relative_yaw_);
  action_server_->publish_feedback(feedback_);

  const double remaining_yaw = std::fabs(cmd_yaw_) - std::fabs(relative_yaw_);
  if (remaining_yaw <= 1e-3) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = geometry_msgs::msg::Twist();
  const double target_vel = std::sqrt(2.0 * rotational_acc_lim_ * remaining_yaw);
  cmd_vel.angular.z = std::copysign(
    std::clamp(target_vel, min_rotational_vel_, max_rotational_vel_), cmd_yaw_);

  if (!isCollisionFree(remaining_yaw, cmd_vel)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Guarded spin rejected an unsafe rotation sweep.");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

bool GuardedSpin::isCollisionFree(double remaining_yaw, const geometry_msgs::msg::Twist & cmd_vel)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return false;
  }
  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);
  return simulateCollisionFree(
    collision_checker_, pose, cmd_vel, cycle_frequency_, simulate_ahead_time_,
    remaining_yaw, false);
}

GuardedBackUp::GuardedBackUp()
: nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>(),
  feedback_(std::make_shared<nav2_msgs::action::BackUp::Feedback>())
{
}

void GuardedBackUp::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(1.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  declareMotionLimits(
    node, behavior_name_, acceleration_limit_, deceleration_limit_, minimum_speed_);
}

nav2_behaviors::Status GuardedBackUp::onRun(
  const std::shared_ptr<const nav2_msgs::action::BackUp::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_ERROR(logger_, "Guarded backup only accepts x-axis backup goals.");
    return nav2_behaviors::Status::FAILED;
  }
  command_distance_ = std::fabs(command->target.x);
  command_speed_ = -std::fabs(command->speed);
  last_vel_ = 0.0;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Guarded backup cannot get the robot pose.");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = command_speed_;
  if (!isCollisionFree(0.0, cmd_vel)) {
    RCLCPP_WARN(logger_, "Guarded backup rejected an occupied rear path.");
    return nav2_behaviors::Status::FAILED;
  }
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status GuardedBackUp::onCycleUpdate()
{
  if ((end_time_ - clock_->now()).seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return nav2_behaviors::Status::FAILED;
  }

  const double traveled = distanceFromStart(initial_pose_, current_pose);
  feedback_->distance_traveled = static_cast<float>(traveled);
  action_server_->publish_feedback(feedback_);
  if (traveled >= command_distance_) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.x = nextVelocity(command_distance_ - traveled);
  if (!isCollisionFree(traveled, cmd_vel)) {
    stopRobot();
    return nav2_behaviors::Status::FAILED;
  }
  last_vel_ = cmd_vel.linear.x;
  vel_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

void GuardedBackUp::onActionCompletion()
{
  last_vel_ = 0.0;
}

double GuardedBackUp::nextVelocity(double remaining_distance)
{
  return limitedVelocity(
    command_speed_, last_vel_, remaining_distance, acceleration_limit_,
    deceleration_limit_, minimum_speed_, cycle_frequency_);
}

bool GuardedBackUp::isCollisionFree(double traveled, const geometry_msgs::msg::Twist & cmd_vel)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return false;
  }
  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);
  return simulateCollisionFree(
    collision_checker_, pose, cmd_vel, cycle_frequency_, simulate_ahead_time_,
    std::max(0.0, command_distance_ - traveled), true);
}

GuardedLateralStep::GuardedLateralStep()
: nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>(),
  feedback_(std::make_shared<nav2_msgs::action::DriveOnHeading::Feedback>())
{
}

void GuardedLateralStep::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(1.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  declareMotionLimits(
    node, behavior_name_, acceleration_limit_, deceleration_limit_, minimum_speed_);
}

nav2_behaviors::Status GuardedLateralStep::onRun(
  const std::shared_ptr<const nav2_msgs::action::DriveOnHeading::Goal> command)
{
  if (command->target.x != 0.0 || command->target.z != 0.0) {
    RCLCPP_ERROR(logger_, "Guarded lateral step only accepts y-axis goals.");
    return nav2_behaviors::Status::FAILED;
  }
  command_distance_ = std::fabs(command->target.y);
  command_speed_ = std::copysign(std::fabs(command->speed), command->target.y);
  last_vel_ = 0.0;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Guarded lateral step cannot get the robot pose.");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.y = command_speed_;
  if (!isCollisionFree(0.0, cmd_vel)) {
    RCLCPP_WARN(logger_, "Guarded lateral step rejected an occupied side path.");
    return nav2_behaviors::Status::FAILED;
  }
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status GuardedLateralStep::onCycleUpdate()
{
  if ((end_time_ - clock_->now()).seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return nav2_behaviors::Status::FAILED;
  }

  const double traveled = distanceFromStart(initial_pose_, current_pose);
  feedback_->distance_traveled = static_cast<float>(traveled);
  action_server_->publish_feedback(feedback_);
  if (traveled >= command_distance_) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.y = nextVelocity(command_distance_ - traveled);
  if (!isCollisionFree(traveled, cmd_vel)) {
    stopRobot();
    return nav2_behaviors::Status::FAILED;
  }
  last_vel_ = cmd_vel.linear.y;
  vel_pub_->publish(std::make_unique<geometry_msgs::msg::Twist>(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

void GuardedLateralStep::onActionCompletion()
{
  last_vel_ = 0.0;
}

double GuardedLateralStep::nextVelocity(double remaining_distance)
{
  return limitedVelocity(
    command_speed_, last_vel_, remaining_distance, acceleration_limit_,
    deceleration_limit_, minimum_speed_, cycle_frequency_);
}

bool GuardedLateralStep::isCollisionFree(double traveled, const geometry_msgs::msg::Twist & cmd_vel)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return false;
  }
  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);
  return simulateCollisionFree(
    collision_checker_, pose, cmd_vel, cycle_frequency_, simulate_ahead_time_,
    std::max(0.0, command_distance_ - traveled), true);
}

}  // namespace robot_nav_recovery

PLUGINLIB_EXPORT_CLASS(robot_nav_recovery::GuardedSpin, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(robot_nav_recovery::GuardedBackUp, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(robot_nav_recovery::GuardedLateralStep, nav2_core::Behavior)
