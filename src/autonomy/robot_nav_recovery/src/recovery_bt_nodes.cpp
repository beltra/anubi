#include "robot_nav_recovery/recovery_bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_nav_recovery
{
namespace
{

rclcpp::QoS costmapQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

CostmapGrid fromMsg(const nav2_msgs::msg::Costmap & msg)
{
  CostmapGrid grid;
  grid.width = msg.metadata.size_x;
  grid.height = msg.metadata.size_y;
  grid.resolution = msg.metadata.resolution;
  grid.origin_x = msg.metadata.origin.position.x;
  grid.origin_y = msg.metadata.origin.position.y;
  grid.origin_yaw = tf2::getYaw(msg.metadata.origin.orientation);
  grid.data = msg.data;
  return grid;
}

template<typename NodeT>
std::string inputOrDefault(NodeT & node, const std::string & key, const std::string & fallback)
{
  std::string value;
  return node.getInput(key, value) ? value : fallback;
}

template<typename NodeT>
double inputOrDefault(NodeT & node, const std::string & key, double fallback)
{
  double value = 0.0;
  return node.getInput(key, value) ? value : fallback;
}

template<typename NodeT>
int inputOrDefault(NodeT & node, const std::string & key, int fallback)
{
  int value = 0;
  return node.getInput(key, value) ? value : fallback;
}

template<typename NodeT>
bool inputOrDefault(NodeT & node, const std::string & key, bool fallback)
{
  bool value = false;
  return node.getInput(key, value) ? value : fallback;
}

template<typename T>
T parameterOrDefault(
  const rclcpp::Node::SharedPtr & node,
  const std::string & key,
  const T & fallback)
{
  T value = fallback;
  if (node->has_parameter(key)) {
    node->get_parameter(key, value);
  }
  return value;
}

std::shared_ptr<tf2_ros::Buffer> blackboardTf(const BT::NodeConfiguration & conf)
{
  try {
    return conf.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  } catch (const std::exception &) {
    return nullptr;
  }
}

bool lookupRobotPose(
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame_id,
  const std::string & robot_base_frame,
  double & robot_x,
  double & robot_y,
  double & robot_yaw)
{
  if (!tf) {
    return false;
  }
  try {
    const auto transform = tf->lookupTransform(
      frame_id, robot_base_frame, tf2::TimePointZero);
    robot_x = transform.transform.translation.x;
    robot_y = transform.transform.translation.y;
    robot_yaw = tf2::getYaw(transform.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

StuckConfig stuckConfigFromPorts(BT::TreeNode & node)
{
  StuckConfig config;
  auto ros_node = node.config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  config.cmd_speed_threshold_mps = parameterOrDefault(
    ros_node, "recovery.stuck_cmd_speed_threshold_mps", config.cmd_speed_threshold_mps);
  config.odom_speed_threshold_mps = parameterOrDefault(
    ros_node, "recovery.stuck_odom_speed_threshold_mps", config.odom_speed_threshold_mps);
  config.stuck_duration_s = parameterOrDefault(
    ros_node, "recovery.stuck_duration_s", config.stuck_duration_s);
  config.cmd_speed_threshold_mps =
    inputOrDefault(node, "cmd_speed_threshold_mps", config.cmd_speed_threshold_mps);
  config.odom_speed_threshold_mps =
    inputOrDefault(node, "odom_speed_threshold_mps", config.odom_speed_threshold_mps);
  config.stuck_duration_s =
    inputOrDefault(node, "stuck_duration_s", config.stuck_duration_s);
  return config;
}

}  // namespace

SelectRecoveryHeading::SelectRecoveryHeading(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = blackboardTf(conf);
  robot_base_frame_ = parameterOrDefault<std::string>(
    node_, "robot_base_frame", "base_footprint");
  robot_base_frame_ = inputOrDefault(*this, "robot_base_frame", robot_base_frame_);
  auto topic = parameterOrDefault<std::string>(
    node_, "recovery.costmap_topic", "local_costmap/costmap_raw");
  topic = inputOrDefault(*this, "costmap_topic", topic);
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    topic, costmapQos(),
    std::bind(&SelectRecoveryHeading::costmapCallback, this, std::placeholders::_1));
}

BT::PortsList SelectRecoveryHeading::providedPorts()
{
  return {
    BT::InputPort<std::string>("costmap_topic", "local_costmap/costmap_raw", "Local costmap topic"),
    BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot base frame"),
    BT::InputPort<int>("sector_count", 8, "Angular sector count"),
    BT::InputPort<double>("inner_radius_m", 0.15, "Minimum sampled radius in meters"),
    BT::InputPort<double>("outer_radius_m", 1.25, "Maximum sampled radius in meters"),
    BT::InputPort<int>("occupied_cost_threshold", 253, "Blocked cost threshold"),
    BT::InputPort<bool>("unknown_is_occupied", false, "Treat unknown cost as blocked"),
    BT::InputPort<double>("blocked_fraction_threshold", 0.2, "Blocked fraction threshold"),
    BT::OutputPort<double>("selected_yaw", "Selected relative yaw in radians")
  };
}

BT::NodeStatus SelectRecoveryHeading::tick()
{
  CostmapGrid grid;
  if (!loadGrid(grid)) {
    return BT::NodeStatus::FAILURE;
  }
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    frame_id = costmap_->header.frame_id;
  }
  if (!loadRobotPose(frame_id, robot_x, robot_y, robot_yaw)) {
    return BT::NodeStatus::FAILURE;
  }

  auto sectors = analyzeSectors(grid, robot_x, robot_y, robot_yaw, loadConfig());
  auto heading = selectEscapeHeading(sectors);
  if (!heading.has_value()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("selected_yaw", heading.value());
  return BT::NodeStatus::SUCCESS;
}

bool SelectRecoveryHeading::loadGrid(CostmapGrid & grid)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) {
    return false;
  }
  grid = fromMsg(*costmap_);
  return isValidGrid(grid);
}

bool SelectRecoveryHeading::loadRobotPose(
  const std::string & frame_id,
  double & robot_x,
  double & robot_y,
  double & robot_yaw)
{
  return lookupRobotPose(tf_, frame_id, robot_base_frame_, robot_x, robot_y, robot_yaw);
}

SectorConfig SelectRecoveryHeading::loadConfig()
{
  SectorConfig config;
  config.sector_count = parameterOrDefault(node_, "recovery.sector_count", config.sector_count);
  config.inner_radius_m = parameterOrDefault(
    node_, "recovery.inner_radius_m", config.inner_radius_m);
  config.outer_radius_m = parameterOrDefault(
    node_, "recovery.outer_radius_m", config.outer_radius_m);
  config.unknown_is_occupied = parameterOrDefault(
    node_, "recovery.unknown_is_occupied", config.unknown_is_occupied);
  config.blocked_fraction_threshold = parameterOrDefault(
    node_, "recovery.blocked_fraction_threshold", config.blocked_fraction_threshold);
  int param_occupied_cost = static_cast<int>(config.occupied_cost_threshold);
  param_occupied_cost = parameterOrDefault(
    node_, "recovery.occupied_cost_threshold", param_occupied_cost);
  config.occupied_cost_threshold =
    static_cast<unsigned char>(std::clamp(param_occupied_cost, 0, 255));
  config.sector_count = inputOrDefault(*this, "sector_count", config.sector_count);
  config.inner_radius_m = inputOrDefault(*this, "inner_radius_m", config.inner_radius_m);
  config.outer_radius_m = inputOrDefault(*this, "outer_radius_m", config.outer_radius_m);
  const int occupied_cost = inputOrDefault(
    *this, "occupied_cost_threshold", static_cast<int>(config.occupied_cost_threshold));
  config.occupied_cost_threshold = static_cast<unsigned char>(std::clamp(occupied_cost, 0, 255));
  config.unknown_is_occupied =
    inputOrDefault(*this, "unknown_is_occupied", config.unknown_is_occupied);
  config.blocked_fraction_threshold =
    inputOrDefault(*this, "blocked_fraction_threshold", config.blocked_fraction_threshold);
  return config;
}

void SelectRecoveryHeading::costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = msg;
}

CheckSectorClear::CheckSectorClear(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = blackboardTf(conf);
  robot_base_frame_ = parameterOrDefault<std::string>(
    node_, "robot_base_frame", "base_footprint");
  robot_base_frame_ = inputOrDefault(*this, "robot_base_frame", robot_base_frame_);
  auto topic = parameterOrDefault<std::string>(
    node_, "recovery.costmap_topic", "local_costmap/costmap_raw");
  topic = inputOrDefault(*this, "costmap_topic", topic);
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    topic, costmapQos(),
    std::bind(&CheckSectorClear::costmapCallback, this, std::placeholders::_1));
}

BT::PortsList CheckSectorClear::providedPorts()
{
  return {
    BT::InputPort<std::string>("sector", "front", "Sector name"),
    BT::InputPort<std::string>("costmap_topic", "local_costmap/costmap_raw", "Local costmap topic"),
    BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot base frame"),
    BT::InputPort<int>("sector_count", 8, "Angular sector count"),
    BT::InputPort<double>("inner_radius_m", 0.15, "Minimum sampled radius in meters"),
    BT::InputPort<double>("outer_radius_m", 1.25, "Maximum sampled radius in meters"),
    BT::InputPort<int>("occupied_cost_threshold", 253, "Blocked cost threshold"),
    BT::InputPort<bool>("unknown_is_occupied", false, "Treat unknown cost as blocked"),
    BT::InputPort<double>("blocked_fraction_threshold", 0.2, "Blocked fraction threshold")
  };
}

BT::NodeStatus CheckSectorClear::tick()
{
  CostmapGrid grid;
  if (!loadGrid(grid)) {
    return BT::NodeStatus::FAILURE;
  }
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    frame_id = costmap_->header.frame_id;
  }
  if (!loadRobotPose(frame_id, robot_x, robot_y, robot_yaw)) {
    return BT::NodeStatus::FAILURE;
  }

  const auto config = loadConfig();
  auto sectors = analyzeSectors(grid, robot_x, robot_y, robot_yaw, config);
  const auto sector = inputOrDefault(*this, "sector", std::string("front"));
  return isSectorClear(sectors, sector, config.sector_count) ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

bool CheckSectorClear::loadGrid(CostmapGrid & grid)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) {
    return false;
  }
  grid = fromMsg(*costmap_);
  return isValidGrid(grid);
}

bool CheckSectorClear::loadRobotPose(
  const std::string & frame_id,
  double & robot_x,
  double & robot_y,
  double & robot_yaw)
{
  return lookupRobotPose(tf_, frame_id, robot_base_frame_, robot_x, robot_y, robot_yaw);
}

SectorConfig CheckSectorClear::loadConfig()
{
  SectorConfig config;
  config.sector_count = parameterOrDefault(node_, "recovery.sector_count", config.sector_count);
  config.inner_radius_m = parameterOrDefault(
    node_, "recovery.inner_radius_m", config.inner_radius_m);
  config.outer_radius_m = parameterOrDefault(
    node_, "recovery.outer_radius_m", config.outer_radius_m);
  config.unknown_is_occupied = parameterOrDefault(
    node_, "recovery.unknown_is_occupied", config.unknown_is_occupied);
  config.blocked_fraction_threshold = parameterOrDefault(
    node_, "recovery.blocked_fraction_threshold", config.blocked_fraction_threshold);
  int param_occupied_cost = static_cast<int>(config.occupied_cost_threshold);
  param_occupied_cost = parameterOrDefault(
    node_, "recovery.occupied_cost_threshold", param_occupied_cost);
  config.occupied_cost_threshold =
    static_cast<unsigned char>(std::clamp(param_occupied_cost, 0, 255));
  config.sector_count = inputOrDefault(*this, "sector_count", config.sector_count);
  config.inner_radius_m = inputOrDefault(*this, "inner_radius_m", config.inner_radius_m);
  config.outer_radius_m = inputOrDefault(*this, "outer_radius_m", config.outer_radius_m);
  const int occupied_cost = inputOrDefault(
    *this, "occupied_cost_threshold", static_cast<int>(config.occupied_cost_threshold));
  config.occupied_cost_threshold = static_cast<unsigned char>(std::clamp(occupied_cost, 0, 255));
  config.unknown_is_occupied =
    inputOrDefault(*this, "unknown_is_occupied", config.unknown_is_occupied);
  config.blocked_fraction_threshold =
    inputOrDefault(*this, "blocked_fraction_threshold", config.blocked_fraction_threshold);
  return config;
}

void CheckSectorClear::costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  costmap_ = msg;
}

MotionHealthy::MotionHealthy(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  detector_(stuckConfigFromPorts(*this))
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto cmd_topic = parameterOrDefault<std::string>(node_, "recovery.cmd_vel_topic", "cmd_vel");
  auto odom_topic = parameterOrDefault<std::string>(node_, "odom_topic", "odom");
  cmd_topic = inputOrDefault(*this, "cmd_vel_topic", cmd_topic);
  odom_topic = inputOrDefault(*this, "odom_topic", odom_topic);
  cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic, rclcpp::QoS(10),
    std::bind(&MotionHealthy::cmdCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10),
    std::bind(&MotionHealthy::odomCallback, this, std::placeholders::_1));
}

BT::PortsList MotionHealthy::providedPorts()
{
  return {
    BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel", "Commanded velocity topic"),
    BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
    BT::InputPort<double>("cmd_speed_threshold_mps", 0.04, "Commanded speed threshold"),
    BT::InputPort<double>("odom_speed_threshold_mps", 0.015, "Observed speed threshold"),
    BT::InputPort<double>("stuck_duration_s", 2.0, "Blocked motion duration")
  };
}

BT::NodeStatus MotionHealthy::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_stuck_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

void MotionHealthy::cmdCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_cmd_ = *msg;
  has_cmd_ = true;
}

void MotionHealthy::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_cmd_) {
    return;
  }
  MotionSample sample;
  sample.time_s = node_->now().seconds();
  sample.cmd_linear_x = last_cmd_.linear.x;
  sample.cmd_linear_y = last_cmd_.linear.y;
  sample.cmd_angular_z = last_cmd_.angular.z;
  sample.odom_linear_x = msg->twist.twist.linear.x;
  sample.odom_linear_y = msg->twist.twist.linear.y;
  sample.odom_angular_z = msg->twist.twist.angular.z;
  is_stuck_ = detector_.update(sample);
}

SetRecoveryContext::SetRecoveryContext(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::PortsList SetRecoveryContext::providedPorts()
{
  return {
    BT::InputPort<std::string>("context", "controller", "Recovery context name")
  };
}

BT::NodeStatus SetRecoveryContext::tick()
{
  const auto context = inputOrDefault(*this, "context", std::string("controller"));
  config().blackboard->set("recovery_context", context);
  return BT::NodeStatus::SUCCESS;
}

RecoveryContextIs::RecoveryContextIs(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

BT::PortsList RecoveryContextIs::providedPorts()
{
  return {
    BT::InputPort<std::string>("context", "controller", "Recovery context name")
  };
}

BT::NodeStatus RecoveryContextIs::tick()
{
  const auto expected = inputOrDefault(*this, "context", std::string("controller"));
  std::string actual;
  try {
    actual = config().blackboard->get<std::string>("recovery_context");
  } catch (const std::exception &) {
    actual = "controller";
  }
  return actual == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

RecoveryDiagnostics::RecoveryDiagnostics(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto topic = parameterOrDefault<std::string>(
    node_, "recovery.diagnostics_topic", "recovery/diagnostics");
  topic = inputOrDefault(*this, "diagnostics_topic", topic);
  pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topic, rclcpp::QoS(10));
}

BT::PortsList RecoveryDiagnostics::providedPorts()
{
  return {
    BT::InputPort<std::string>("diagnostics_topic", "recovery/diagnostics", "Diagnostic topic"),
    BT::InputPort<std::string>("stage", "recovery", "Recovery stage"),
    BT::InputPort<std::string>("message", "recovery event", "Diagnostic message"),
    BT::InputPort<int>("level", 1, "diagnostic_msgs level"),
    BT::InputPort<bool>("fail", false, "Return FAILURE after publishing")
  };
}

BT::NodeStatus RecoveryDiagnostics::tick()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "go2_nav_recovery";
  status.hardware_id = "go2";
  status.level = static_cast<unsigned char>(
    std::clamp(inputOrDefault(*this, "level", 1), 0, 3));
  status.message = inputOrDefault(*this, "message", std::string("recovery event"));

  diagnostic_msgs::msg::KeyValue stage;
  stage.key = "stage";
  stage.value = inputOrDefault(*this, "stage", std::string("recovery"));
  status.values.push_back(stage);

  std::string context = "controller";
  try {
    context = config().blackboard->get<std::string>("recovery_context");
  } catch (const std::exception &) {
  }
  diagnostic_msgs::msg::KeyValue context_value;
  context_value.key = "context";
  context_value.value = context;
  status.values.push_back(context_value);

  array.status.push_back(status);
  pub_->publish(array);
  return inputOrDefault(*this, "fail", false) ?
         BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

GuardedLateralStepAction::GuardedLateralStepAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::DriveOnHeading>(
    xml_tag_name, action_name, conf)
{
}

BT::PortsList GuardedLateralStepAction::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<double>("step_dist", 0.15, "Distance to step laterally"),
      BT::InputPort<double>("step_speed", 0.03, "Lateral step speed"),
      BT::InputPort<std::string>("direction", "left", "left or right"),
      BT::InputPort<double>("time_allowance", 8.0, "Allowed step time")
    });
}

void GuardedLateralStepAction::on_tick()
{
  const double distance = std::fabs(inputOrDefault(*this, "step_dist", 0.15));
  const double speed = std::fabs(inputOrDefault(*this, "step_speed", 0.03));
  const auto direction = inputOrDefault(*this, "direction", std::string("left"));
  const double sign = direction == "right" ? -1.0 : 1.0;
  const double time_allowance = inputOrDefault(*this, "time_allowance", 8.0);

  goal_.target.x = 0.0;
  goal_.target.y = sign * distance;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

EscalatingRecovery::EscalatingRecovery(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode(name, conf)
{
}

BT::PortsList EscalatingRecovery::providedPorts()
{
  return {};
}

BT::NodeStatus EscalatingRecovery::tick()
{
  const size_t child_count = children_nodes_.size();
  while (current_child_idx_ < child_count) {
    auto * child = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child->executeTick();
    if (child_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }

    child->halt();
    current_child_idx_ += 1;
    if (child_status == BT::NodeStatus::SUCCESS) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  current_child_idx_ = 0;
  return BT::NodeStatus::FAILURE;
}

void EscalatingRecovery::halt()
{
  haltChildren();
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace robot_nav_recovery

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_nav_recovery::SelectRecoveryHeading>("SelectRecoveryHeading");
  factory.registerNodeType<robot_nav_recovery::CheckSectorClear>("CheckSectorClear");
  factory.registerNodeType<robot_nav_recovery::MotionHealthy>("MotionHealthy");
  factory.registerNodeType<robot_nav_recovery::SetRecoveryContext>("SetRecoveryContext");
  factory.registerNodeType<robot_nav_recovery::RecoveryContextIs>("RecoveryContextIs");
  factory.registerNodeType<robot_nav_recovery::RecoveryDiagnostics>("RecoveryDiagnostics");
  factory.registerNodeType<robot_nav_recovery::EscalatingRecovery>("EscalatingRecovery");

  BT::NodeBuilder lateral_builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robot_nav_recovery::GuardedLateralStepAction>(
        name, "guarded_lateral_step", config);
    };
  factory.registerBuilder<robot_nav_recovery::GuardedLateralStepAction>(
    "GuardedLateralStep", lateral_builder);
}
