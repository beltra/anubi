/**
 * go2_hw_bridge.cpp
 *
 * Central hardware bridge between the Go2 robot and the Nav2 navigation stack.
 * This is the only node in the stack that speaks the Unitree Go2 protocol.
 *
 * ── Subscriptions ────────────────────────────────────────────────────────────
 *  /sportmodestate  [unitree_go/SportModeState]
 *    Published by the Go2 internal ROS2 bridge at ~50 Hz.
 *    Contains proprioceptive state: position, velocity, IMU, battery.
 *
 *  /cmd_vel  [geometry_msgs/Twist]
 *    Velocity commands from Nav2's controller_server.
 *
 * ── Publications ─────────────────────────────────────────────────────────────
 *  /odom              [nav_msgs/Odometry]       — re-published for Nav2
 *  /imu/data          [sensor_msgs/Imu]         — raw body IMU for fusion / deskew
 *  /joint_states      [sensor_msgs/JointState]  — 12 leg joints @ ~50 Hz from
 *                                                  /lowstate.motor_state[0..11]
 *                                                  (decimated from ~500 Hz raw)
 *  /robot_status      [robot_common_interfaces/RobotStatus] — health at 1 Hz
 *  /api/sport/request [unitree_api/Request]     — motion commands to Go2
 *  TF: odom → base_footprint  (planar x, y, yaw odometry)
 *  TF: base_footprint → base_link  (body roll / pitch + standing height)
 *
 * ── TF Tree ──────────────────────────────────────────────────────────────────
 *  map  ──(slam_toolbox)──►  odom  ──(this node)──►  base_footprint
 *                                        ├─────────►  base_link
 *                                        └─(launch)►  lidar_stabilized
 *  base_link ──(URDF)──►  lidar_link
 *           └─(URDF)──►  imu_link
 *
 * ── Design Notes ─────────────────────────────────────────────────────────────
 *  - The Go2 SportModeState.position is in the Go2's body-initialised internal
 *    frame.  We treat this frame as our 'odom' frame (short-term accurate,
 *    accumulates drift over time — slam_toolbox corrects this via map→odom).
 *  - Z is zeroed in the odom→base_footprint transform and only yaw is published
 *    there. Roll/pitch are separated into base_footprint→base_link so Nav2 sees
 *    a planar base frame while perception can still access body attitude.
 *  - A cmd_vel watchdog publishes StopMove if no command arrives within
 *    cmd_timeout seconds.  This prevents the robot from running away if
 *    Nav2 crashes.
 *  - The node clamps velocities to safe limits before forwarding to the robot.
 *
 * For non-Go2 robots, keep the same external contract:
 *   subscribe to robot state, publish odom + TF, subscribe to cmd_vel,
 *   then forward velocity commands via the robot-specific motion API.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "robot_common_interfaces/msg/robot_status.hpp"
#include "go2_bringup/sport_client.hpp"

namespace go2_bringup
{

class Go2HwBridge : public rclcpp::Node
{
public:
  Go2HwBridge()
  : Node("go2_hw_bridge")
  {
    // ── Parameters ────────────────────────────────────────────────────────
    declare_parameter<std::string>("state_topic",      "/sportmodestate");
    declare_parameter<std::string>("lowstate_topic",   "/lowstate");
    declare_parameter<std::string>("cmd_vel_topic",    "/cmd_vel");
    declare_parameter<std::string>("sport_req_topic",  "/api/sport/request");
    declare_parameter<std::string>("odom_topic",       "/odom");
    declare_parameter<std::string>("imu_topic",        "/imu/data");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<double>("joint_states_rate",     50.0);   // [Hz] decimation from ~500 Hz lowstate
    declare_parameter<std::string>("odom_frame",       "odom");
    declare_parameter<std::string>("base_frame",       "base_footprint");
    declare_parameter<std::string>("base_link_frame",  "base_link");
    declare_parameter<std::string>("imu_frame",        "imu_link");
    declare_parameter<bool>("publish_tf",              true);
    declare_parameter<bool>("publish_body_tf",         true);
    declare_parameter<double>("standing_height",       0.28);
    declare_parameter<double>("cmd_timeout",           0.5);   // [s] watchdog
    declare_parameter<double>("max_linear_vel",        1.0);   // [m/s]
    declare_parameter<double>("max_lateral_vel",       0.3);   // [m/s]
    declare_parameter<double>("max_angular_vel",       1.5);   // [rad/s]
    declare_parameter<bool>("enable_startup_motion_sequence", true);
    declare_parameter<bool>("enable_goal_motion_sequence", true);
    declare_parameter<double>("startup_standup_delay_sec", 1.0);
    declare_parameter<double>("startup_classic_walk_delay_sec", 3.0);
    declare_parameter<std::string>(
      "nav_goal_status_topic", "/navigate_to_pose/_action/status");

    state_topic_    = get_parameter("state_topic").as_string();
    lowstate_topic_ = get_parameter("lowstate_topic").as_string();
    cmd_vel_topic_  = get_parameter("cmd_vel_topic").as_string();
    sport_topic_    = get_parameter("sport_req_topic").as_string();
    odom_topic_     = get_parameter("odom_topic").as_string();
    imu_topic_      = get_parameter("imu_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    joint_states_rate_  = get_parameter("joint_states_rate").as_double();
    odom_frame_     = get_parameter("odom_frame").as_string();
    base_frame_     = get_parameter("base_frame").as_string();
    base_link_frame_ = get_parameter("base_link_frame").as_string();
    imu_frame_      = get_parameter("imu_frame").as_string();
    publish_tf_     = get_parameter("publish_tf").as_bool();
    publish_body_tf_ = get_parameter("publish_body_tf").as_bool();
    standing_height_ = get_parameter("standing_height").as_double();
    cmd_timeout_    = get_parameter("cmd_timeout").as_double();
    max_lin_        = get_parameter("max_linear_vel").as_double();
    max_lat_        = get_parameter("max_lateral_vel").as_double();
    max_ang_        = get_parameter("max_angular_vel").as_double();
    enable_startup_motion_sequence_ =
      get_parameter("enable_startup_motion_sequence").as_bool();
    enable_goal_motion_sequence_ =
      get_parameter("enable_goal_motion_sequence").as_bool();
    startup_standup_delay_sec_ = get_parameter("startup_standup_delay_sec").as_double();
    startup_classic_walk_delay_sec_ =
      get_parameter("startup_classic_walk_delay_sec").as_double();
    nav_goal_status_topic_ = get_parameter("nav_goal_status_topic").as_string();

    // ── Publishers ────────────────────────────────────────────────────────
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_pub_   = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    imu_pub_    = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS(50));
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<robot_common_interfaces::msg::RobotStatus>(
      "/robot_status", rclcpp::QoS(10).transient_local());
    sport_pub_  = create_publisher<unitree_api::msg::Request>(
      sport_topic_, rclcpp::QoS(10));

    // ── Subscriptions ─────────────────────────────────────────────────────
    state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      state_topic_, rclcpp::QoS(10),
      std::bind(&Go2HwBridge::on_sport_state, this, std::placeholders::_1));

    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
      lowstate_topic_, rclcpp::QoS(10),
      std::bind(&Go2HwBridge::on_low_state, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&Go2HwBridge::on_cmd_vel, this, std::placeholders::_1));

    nav_goal_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      nav_goal_status_topic_, rclcpp::QoS(10),
      std::bind(&Go2HwBridge::on_nav_goal_status, this, std::placeholders::_1));

    // ── Watchdog timer ────────────────────────────────────────────────────
    // Fires at 10 Hz.  If cmd_vel goes stale, sends StopMove to the robot.
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Go2HwBridge::on_watchdog, this));

    // ── Status timer ─────────────────────────────────────────────────────
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&Go2HwBridge::publish_status, this));

    // ── Joint-state timer ────────────────────────────────────────────────
    // Decimates /lowstate (~500 Hz) down to joint_states_rate_ to avoid
    // saturating TF / robot_state_publisher with redundant updates.
    if (joint_states_rate_ > 0.0) {
      const auto js_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / joint_states_rate_));
      joint_states_timer_ = create_wall_timer(
        js_period, std::bind(&Go2HwBridge::publish_joint_states, this));
    }

    // Startup sequence (real robot): StandUp first, then enable ClassicWalk.
    if (enable_startup_motion_sequence_ || enable_goal_motion_sequence_) {
      startup_motion_sequence_requested_ = enable_startup_motion_sequence_;
      startup_sequence_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&Go2HwBridge::run_motion_sequence, this));
      RCLCPP_INFO(
        get_logger(),
        "  motion sequences enabled: RecoveryStand -> StandUp at +%.1fs -> ClassicWalk at +%.1fs",
        startup_standup_delay_sec_, startup_classic_walk_delay_sec_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Go2HwBridge ready.  state=%s  lowstate=%s  cmd=%s  odom=%s  imu=%s  sport=%s",
      state_topic_.c_str(), lowstate_topic_.c_str(),
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), imu_topic_.c_str(), sport_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "  limits: lin=%.2f lat=%.2f ang=%.2f  watchdog=%.1fs  publish_tf=%s  publish_body_tf=%s",
      max_lin_, max_lat_, max_ang_, cmd_timeout_,
      publish_tf_ ? "true" : "false", publish_body_tf_ ? "true" : "false");
  }

private:
  // ── Sport state callback ─────────────────────────────────────────────────

  void on_sport_state(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    // Validate the Go2 hardware clock against a minimum wall-clock epoch.
    // The Go2 clock is a wall clock when NTP-synced, but during early boot it
    // may report device uptime (e.g. 300 s since boot → Jan 1970 + 5 min),
    // which would corrupt TF/SLAM timestamps.
    // kMinEpochSec = 2020-01-01T00:00:00Z.  Uptime values are always smaller.
    static constexpr int64_t kMinEpochSec = 1577836800LL;
    rclcpp::Time stamp;
    if (static_cast<int64_t>(msg->stamp.sec) >= kMinEpochSec) {
      stamp = rclcpp::Time(
        static_cast<int32_t>(msg->stamp.sec),
        msg->stamp.nanosec,
        RCL_ROS_TIME);
    } else {
      stamp = now();
    }

    tf2::Quaternion imu_orientation(
      static_cast<double>(msg->imu_state.quaternion[1]),
      static_cast<double>(msg->imu_state.quaternion[2]),
      static_cast<double>(msg->imu_state.quaternion[3]),
      static_cast<double>(msg->imu_state.quaternion[0]));
    imu_orientation.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(imu_orientation).getRPY(roll, pitch, yaw);

    tf2::Quaternion planar_orientation;
    planar_orientation.setRPY(0.0, 0.0, yaw);

    tf2::Quaternion body_orientation;
    body_orientation.setRPY(roll, pitch, 0.0);

    // ── Odometry message ──────────────────────────────────────────────────
    // The Go2's SportModeState.position is in the robot's internal odometry frame.
    // We treat this as our 'odom' frame.
    // Z = 0 and yaw-only: 2D navigation convention for Nav2 / SLAM.
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;

    odom.pose.pose.position.x = static_cast<double>(msg->position[0]);
    odom.pose.pose.position.y = static_cast<double>(msg->position[1]);
    odom.pose.pose.position.z = 0.0;  // 2D: always zero

    odom.pose.pose.orientation = tf2::toMsg(planar_orientation);
    // Covariance is conservative and manually tuned for 2D navigation.
    // X/Y position: 0.25 m² (~0.5 m σ) reflects typical drift under Sport Mode odometry.
    // Yaw: 0.017 rad² (~7.5° σ) reflects IMU heading accuracy.
    // Z/roll/pitch: 1e6 (unconstrained) — 2D navigation; these DOFs are not tracked.
    odom.pose.covariance[0] = 0.25;
    odom.pose.covariance[7] = 0.25;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 0.017;

    // Body-frame velocities from Go2 state
    odom.twist.twist.linear.x  = static_cast<double>(msg->velocity[0]);
    odom.twist.twist.linear.y  = static_cast<double>(msg->velocity[1]);
    odom.twist.twist.linear.z  = 0.0;
    odom.twist.twist.angular.z = static_cast<double>(msg->yaw_speed);
    odom.twist.covariance[0] = 0.3;
    odom.twist.covariance[7] = 0.3;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 0.017;

    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation = tf2::toMsg(imu_orientation);
    imu.angular_velocity.x = static_cast<double>(msg->imu_state.gyroscope[0]);
    imu.angular_velocity.y = static_cast<double>(msg->imu_state.gyroscope[1]);
    imu.angular_velocity.z = static_cast<double>(msg->imu_state.gyroscope[2]);
    imu.linear_acceleration.x = static_cast<double>(msg->imu_state.accelerometer[0]);
    imu.linear_acceleration.y = static_cast<double>(msg->imu_state.accelerometer[1]);
    imu.linear_acceleration.z = static_cast<double>(msg->imu_state.accelerometer[2]);
    imu.orientation_covariance[0] = 0.01;
    imu.orientation_covariance[4] = 0.01;
    imu.orientation_covariance[8] = 0.02;
    imu.angular_velocity_covariance[0] = 0.001;
    imu.angular_velocity_covariance[4] = 0.001;
    imu.angular_velocity_covariance[8] = 0.001;
    imu.linear_acceleration_covariance[0] = 0.02;
    imu.linear_acceleration_covariance[4] = 0.02;
    imu.linear_acceleration_covariance[8] = 0.02;
    imu_pub_->publish(imu);

    if (publish_tf_) {
      // ── TF: odom → base_footprint ──────────────────────────────────────
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id  = base_frame_;

      tf.transform.translation.x = odom.pose.pose.position.x;
      tf.transform.translation.y = odom.pose.pose.position.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation      = tf2::toMsg(planar_orientation);

      tf_broadcaster_->sendTransform(tf);
    }

    if (publish_body_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = base_frame_;
      tf.child_frame_id = base_link_frame_;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = standing_height_;
      tf.transform.rotation = tf2::toMsg(body_orientation);

      tf_broadcaster_->sendTransform(tf);
    }

    // Cache state for status publisher
    last_state_ = *msg;
    has_state_  = true;
  }

  // ── LowState callback (battery data + joint state cache) ─────────────────

  void on_low_state(const unitree_go::msg::LowState::SharedPtr msg)
  {
    last_low_state_ = *msg;
    has_low_state_  = true;
  }

  // ── Joint state publisher ────────────────────────────────────────────────
  // Maps Unitree Go2 motor indices to CHAMP-style URDF joint names.  Index
  // order follows unitree_sdk2/include/unitree/dds_wrapper/robots/go2/defines.h
  // (FR=0..2, FL=3..5, RR=6..8, RL=9..11 with 0=hip, 1=thigh, 2=calf).
  // Joint names match leg.xacro in go2_description.
  static constexpr std::array<const char *, 12> kJointNames = {
    "rf_hip_joint",        "rf_upper_leg_joint",  "rf_lower_leg_joint",   // FR
    "lf_hip_joint",        "lf_upper_leg_joint",  "lf_lower_leg_joint",   // FL
    "rh_hip_joint",        "rh_upper_leg_joint",  "rh_lower_leg_joint",   // RR
    "lh_hip_joint",        "lh_upper_leg_joint",  "lh_lower_leg_joint"};  // RL

  void publish_joint_states()
  {
    if (!has_low_state_) {
      return;  // No data yet; robot_state_publisher will hold last known transforms.
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.assign(kJointNames.begin(), kJointNames.end());
    js.position.resize(12);
    js.velocity.resize(12);
    js.effort.resize(12);
    for (size_t i = 0; i < 12; ++i) {
      const auto & m = last_low_state_.motor_state[i];
      js.position[i] = static_cast<double>(m.q);
      js.velocity[i] = static_cast<double>(m.dq);
      js.effort[i]   = static_cast<double>(m.tau_est);
    }
    joint_states_pub_->publish(js);
  }

  // ── cmd_vel callback ─────────────────────────────────────────────────────

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();

    // Clamp to safe limits
    const float vx   = clamp(static_cast<float>(msg->linear.x),  -max_lin_, max_lin_);
    const float vy   = clamp(static_cast<float>(msg->linear.y),  -max_lat_, max_lat_);
    const float vyaw = clamp(static_cast<float>(msg->angular.z), -max_ang_, max_ang_);

    if (motion_sequence_active_ && !is_zero_command(vx, vy, vyaw)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Holding nonzero cmd_vel while motion sequence is active");
      publish_stop_move();
      watchdog_stop_active_ = false;
      return;
    }

    if (is_zero_command(vx, vy, vyaw)) {
      publish_stop_move();
      watchdog_stop_active_ = false;
      return;
    }

    unitree_api::msg::Request req;
    sport_.Move(req, vx, vy, vyaw);
    sport_pub_->publish(req);
    last_command_was_stop_ = false;
    watchdog_stop_active_ = false;
  }

  void on_nav_goal_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    if (!enable_goal_motion_sequence_ || goal_motion_sequence_requested_ ||
      goal_motion_sequence_completed_)
    {
      return;
    }

    for (const auto & status : msg->status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        goal_motion_sequence_requested_ = true;
        RCLCPP_INFO(
          get_logger(),
          "First navigation goal accepted: queued RecoveryStand -> StandUp -> ClassicWalk");
        return;
      }
    }
  }

  // ── Watchdog ──────────────────────────────────────────────────────────────

  void on_watchdog()
  {
    if (!last_cmd_time_.has_value()) {
      return;  // No command ever received; don't send spurious stops
    }

    const double age = (now() - *last_cmd_time_).seconds();
    if (age > cmd_timeout_) {
      if (!watchdog_stop_active_) {
        publish_stop_move();
        watchdog_stop_active_ = true;
        RCLCPP_WARN(get_logger(),
          "cmd_vel stale (%.2fs > %.2fs) — sent StopMove", age, cmd_timeout_);
      }
    } else {
      watchdog_stop_active_ = false;
    }
  }

  // ── Motion sequence ───────────────────────────────────────────────────────

  void run_motion_sequence()
  {
    if (!has_state_) {
      return;
    }

    // Avoid dropping startup commands before DDS discovery settles.
    if (sport_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for a subscriber on %s before sending startup Sport API commands",
        sport_topic_.c_str());
      return;
    }

    if (!motion_sequence_active_) {
      if (startup_motion_sequence_requested_) {
        start_motion_sequence(MotionSequenceTrigger::Startup, "startup");
      } else if (goal_motion_sequence_requested_) {
        start_motion_sequence(MotionSequenceTrigger::GoalAccepted, "first accepted goal");
      }
      return;
    }

    if (!motion_sequence_recovery_sent_) {
      unitree_api::msg::Request req;
      sport_.RecoveryStand(req);
      sport_pub_->publish(req);
      last_command_was_stop_ = false;
      motion_sequence_recovery_sent_ = true;
      motion_sequence_recovery_time_ = now();
      RCLCPP_INFO(get_logger(), "Sent motion sequence Sport API command: RecoveryStand");
      return;
    }

    if (!motion_sequence_standup_sent_) {
      const double since_recovery = (now() - *motion_sequence_recovery_time_).seconds();
      if (since_recovery >= startup_standup_delay_sec_) {
        unitree_api::msg::Request req;
        sport_.StandUp(req);
        sport_pub_->publish(req);
        last_command_was_stop_ = false;
        motion_sequence_standup_sent_ = true;
        motion_sequence_standup_time_ = now();
        RCLCPP_INFO(get_logger(), "Sent motion sequence Sport API command: StandUp");
      }
      return;
    }

    if (!motion_sequence_classic_walk_sent_) {
      const double since_standup = (now() - *motion_sequence_standup_time_).seconds();
      if (since_standup >= startup_classic_walk_delay_sec_) {
        unitree_api::msg::Request req;
        sport_.ClassicWalk(req, true);
        sport_pub_->publish(req);
        last_command_was_stop_ = false;
        motion_sequence_classic_walk_sent_ = true;
        RCLCPP_INFO(get_logger(), "Sent motion sequence Sport API command: ClassicWalk(true)");
        finish_motion_sequence();
      }
    }
  }

  // ── Status publisher ──────────────────────────────────────────────────────

  void publish_status()
  {
    robot_common_interfaces::msg::RobotStatus status;
    status.header.stamp = now();

    if (has_state_) {
      status.hw_ok       = true;
      status.motion_mode = std::to_string(last_state_.mode);
    } else {
      status.hw_ok = false;
    }

    // Battery state comes from /lowstate (BmsState), not from SportModeState.
    // Publish 0 / warn once until the first LowState message arrives.
    if (has_low_state_) {
      // BmsState.soc is integer [0..100 %] → normalise to [0.0..1.0]
      status.battery_soc     = static_cast<float>(last_low_state_.bms_state.soc) / 100.0f;
      status.battery_voltage = last_low_state_.power_v;
    } else {
      status.battery_soc     = 0.0f;
      status.battery_voltage = 0.0f;
      if (has_state_) {
        // Warn once: LowState not yet received (topic may not be published).
        RCLCPP_WARN_ONCE(get_logger(),
          "No /lowstate received yet — battery_soc and battery_voltage are 0 "
          "(check that the Go2 bridge publishes %s)", lowstate_topic_.c_str());
      }
    }
    status.estop_active = watchdog_stop_active_;

    status_pub_->publish(status);
  }

  // ── Utilities ─────────────────────────────────────────────────────────────

  static float clamp(float v, double lo, double hi)
  {
    return static_cast<float>(std::max(lo, std::min(hi, static_cast<double>(v))));
  }

  static bool is_zero_command(float vx, float vy, float vyaw)
  {
    return std::abs(vx) < 1e-4f && std::abs(vy) < 1e-4f && std::abs(vyaw) < 1e-4f;
  }

  void publish_stop_move()
  {
    if (last_command_was_stop_) {
      return;
    }

    unitree_api::msg::Request req;
    sport_.StopMove(req);
    sport_pub_->publish(req);
    last_command_was_stop_ = true;
  }

  enum class MotionSequenceTrigger
  {
    None,
    Startup,
    GoalAccepted,
  };

  void start_motion_sequence(MotionSequenceTrigger trigger, const std::string & reason)
  {
    motion_sequence_trigger_ = trigger;
    motion_sequence_active_ = true;
    motion_sequence_recovery_sent_ = false;
    motion_sequence_standup_sent_ = false;
    motion_sequence_classic_walk_sent_ = false;
    motion_sequence_recovery_time_.reset();
    motion_sequence_standup_time_.reset();
    publish_stop_move();
    RCLCPP_INFO(
      get_logger(),
      "Starting motion sequence for %s: RecoveryStand -> StandUp -> ClassicWalk",
      reason.c_str());
  }

  void finish_motion_sequence()
  {
    if (motion_sequence_trigger_ == MotionSequenceTrigger::Startup) {
      startup_motion_sequence_requested_ = false;
      startup_motion_sequence_completed_ = true;
    } else if (motion_sequence_trigger_ == MotionSequenceTrigger::GoalAccepted) {
      goal_motion_sequence_requested_ = false;
      goal_motion_sequence_completed_ = true;
    }

    motion_sequence_active_ = false;
    motion_sequence_trigger_ = MotionSequenceTrigger::None;
  }

  // ── Members ───────────────────────────────────────────────────────────────

  std::string state_topic_;
  std::string lowstate_topic_;
  std::string cmd_vel_topic_;
  std::string sport_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string joint_states_topic_;
  double joint_states_rate_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string base_link_frame_;
  std::string imu_frame_;
  bool publish_tf_;
  bool publish_body_tf_;
  double standing_height_;
  double cmd_timeout_;
  double max_lin_, max_lat_, max_ang_;
  bool enable_startup_motion_sequence_;
  bool enable_goal_motion_sequence_;
  double startup_standup_delay_sec_;
  double startup_classic_walk_delay_sec_;
  std::string nav_goal_status_topic_;

  SportClient sport_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<robot_common_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_goal_status_sub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr joint_states_timer_;
  rclcpp::TimerBase::SharedPtr startup_sequence_timer_;

  std::optional<rclcpp::Time> last_cmd_time_;
  std::optional<rclcpp::Time> motion_sequence_recovery_time_;
  std::optional<rclcpp::Time> motion_sequence_standup_time_;
  bool watchdog_stop_active_ = false;
  bool last_command_was_stop_ = false;
  bool startup_motion_sequence_requested_ = false;
  bool startup_motion_sequence_completed_ = false;
  bool goal_motion_sequence_requested_ = false;
  bool goal_motion_sequence_completed_ = false;
  bool motion_sequence_active_ = false;
  bool motion_sequence_recovery_sent_ = false;
  bool motion_sequence_standup_sent_ = false;
  bool motion_sequence_classic_walk_sent_ = false;
  MotionSequenceTrigger motion_sequence_trigger_ = MotionSequenceTrigger::None;
  bool has_state_     = false;
  bool has_low_state_ = false;
  unitree_go::msg::SportModeState last_state_;
  unitree_go::msg::LowState       last_low_state_;
};

}  // namespace go2_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_bringup::Go2HwBridge>());
  rclcpp::shutdown();
  return 0;
}
