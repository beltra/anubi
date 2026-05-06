#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "robot_nav_recovery/costmap_sector_analyzer.hpp"
#include "robot_nav_recovery/motion_stuck_detector.hpp"

namespace robot_nav_recovery
{

class SelectRecoveryHeading : public BT::SyncActionNode
{
public:
  SelectRecoveryHeading(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  bool loadGrid(CostmapGrid & grid);
  bool loadRobotPose(
    const std::string & frame_id,
    double & robot_x,
    double & robot_y,
    double & robot_yaw);
  SectorConfig loadConfig();
  void costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
  mutable std::mutex costmap_mutex_;
  std::string robot_base_frame_;
};

class CheckSectorClear : public BT::ConditionNode
{
public:
  CheckSectorClear(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  bool loadGrid(CostmapGrid & grid);
  bool loadRobotPose(
    const std::string & frame_id,
    double & robot_x,
    double & robot_y,
    double & robot_yaw);
  SectorConfig loadConfig();
  void costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
  mutable std::mutex costmap_mutex_;
  std::string robot_base_frame_;
};

class MotionHealthy : public BT::ConditionNode
{
public:
  MotionHealthy(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  void cmdCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  MotionStuckDetector detector_;
  geometry_msgs::msg::Twist last_cmd_;
  bool has_cmd_{false};
  bool is_stuck_{false};
  mutable std::mutex mutex_;
};

class SetRecoveryContext : public BT::SyncActionNode
{
public:
  SetRecoveryContext(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
};

class RecoveryContextIs : public BT::ConditionNode
{
public:
  RecoveryContextIs(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
};

class RecoveryDiagnostics : public BT::SyncActionNode
{
public:
  RecoveryDiagnostics(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
};

class GuardedLateralStepAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
  GuardedLateralStepAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts();
};

class EscalatingRecovery : public BT::ControlNode
{
public:
  EscalatingRecovery(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts();

private:
  size_t current_child_idx_{0};
};

}  // namespace robot_nav_recovery
