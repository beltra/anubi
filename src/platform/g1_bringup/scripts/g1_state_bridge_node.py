#!/usr/bin/env python3
"""g1_state_bridge: publish the Unitree G1 proprioceptive state to ROS for the EKF.

The G1 analogue of go2_hw_bridge's STATE half. go2_hw_bridge speaks the Go2 Sport
API over ROS; the G1 instead speaks Unitree DDS, and the COMMAND half is handled
separately by cmd_vel_to_amo (Nav2 -> AMO WebSocket). This node only READS the
robot and publishes what the EKF / perception stack needs:

    DDS rt/odommodestate (SportModeState_) --> /odom_raw   (nav_msgs/Odometry)
    DDS rt/lowstate      (LowState_ IMU)    --> /imu/data   (sensor_msgs/Imu)
    TF base_footprint -> base_link          (body roll/pitch + standing height)
    TF odom -> base_footprint               (ONLY when publish_tf:=true, i.e. no EKF)

It never publishes rt/lowcmd: the AMO driver (g1_locomotion amo_driver, a separate
process) owns motion. This node reuses g1_locomotion's proven G1DDSInterface in a
READ-ONLY way (connect() brings up the subscribers; start_publisher() is never
called, so no joint command thread runs).

Frame contract (matches go2_hw_bridge, see docs/topic_map.md)
------------------------------------------------------------
    map ->(slam)-> odom ->(EKF)-> base_footprint ->(this node)-> base_link(==pelvis)
The EKF owns odom -> base_footprint; this node owns base_footprint -> base_link so
Nav2 sees a planar base while perception keeps body attitude. With publish_tf:=true
(EKF disabled) this node also publishes the planar odom -> base_footprint TF.

Robustness
----------
The pelvis IMU (rt/lowstate) is always available while the robot is powered, so
/imu/data and the body TF are always published. The body odometry (rt/odommodestate)
may be stale or absent depending on the active control mode; /odom_raw is published
only while it is fresh, and the EKF tolerates its absence by falling back to DLIO
(use_lio:=true). See docs/g1.md.

Run standalone (params otherwise come from config/g1_params.yaml):
    ros2 run g1_bringup g1_state_bridge --ros-args -p net_if:=eth0
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def _quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Yaw (rad) from a wxyz quaternion (ZYX convention)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_to_roll_pitch(qw: float, qx: float, qy: float, qz: float):
    """Roll, pitch (rad) from a wxyz quaternion (ZYX convention)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


def _yaw_to_quat(yaw: float):
    """xyzw quaternion for a yaw-only rotation."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    """xyzw quaternion from roll, pitch, yaw (ZYX)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class G1StateBridge(Node):
    def __init__(self):
        super().__init__("g1_state_bridge")

        # DDS NIC the Unitree robot is reachable on (e.g. eth0). Empty -> SDK default.
        self.declare_parameter("net_if", "")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("imu_frame", "imu")
        # publish_tf: planar odom->base_footprint here. Leave FALSE when the EKF
        # owns that TF (the default real path). publish_body_tf: base_footprint->
        # base_link with body roll/pitch (owned here, the EKF does not produce it).
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_body_tf", True)
        # Nominal pelvis(base_link) height above the floor while standing [m].
        # Must match g1_description standing_height. Calibrate on hardware.
        self.declare_parameter("standing_height", 0.78)
        self.declare_parameter("rate_hz", 50.0)
        # Drop /odom_raw if the DDS body odometry has not refreshed within this many
        # seconds (control-mode dependent). The body TF + IMU keep publishing.
        self.declare_parameter("body_odom_timeout", 0.3)

        net_if = self.get_parameter("net_if").get_parameter_value().string_value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._imu_topic = self.get_parameter("imu_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._base_link_frame = self.get_parameter("base_link_frame").value
        self._imu_frame = self.get_parameter("imu_frame").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._publish_body_tf = bool(self.get_parameter("publish_body_tf").value)
        self._standing_height = float(self.get_parameter("standing_height").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._odom_timeout = float(self.get_parameter("body_odom_timeout").value)

        # Sensor-data QoS for the IMU stream; reliable for odom.
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, self._imu_topic, imu_qos)
        self._tf = TransformBroadcaster(self)

        # Reuse g1_locomotion's proven DDS transport, READ-ONLY: connect() starts the
        # rt/lowstate + rt/odommodestate subscribers; we never call start_publisher(),
        # so no rt/lowcmd command thread runs here (the AMO driver owns commands).
        try:
            from g1_locomotion.robot_dds_io import G1DDSInterface
        except Exception as exc:  # pragma: no cover - hardware/runtime only
            self.get_logger().fatal(
                "could not import g1_locomotion.robot_dds_io (%s); g1_state_bridge "
                "needs g1_locomotion + unitree_sdk2py and a real robot." % exc)
            raise

        self._io = G1DDSInterface(net_if=net_if or None, observe_only=True)
        try:
            self._io.connect()
        except Exception as exc:  # pragma: no cover - hardware/runtime only
            self.get_logger().fatal(
                "DDS connect failed (%s). Check net_if and that the G1 is powered "
                "and on-network." % exc)
            raise
        self.get_logger().info(
            "g1_state_bridge connected (net_if=%r). Waiting for first lowstate..."
            % (net_if or "<default>"))
        # Block briefly for the first state so the IMU/TF are valid from the first tick.
        self._io.wait_for_state(timeout=10.0)

        self.create_timer(1.0 / max(1.0, rate_hz), self._tick)
        self.get_logger().info(
            "publishing odom=%s imu=%s body_tf=%s odom_tf=%s standing_height=%.2f"
            % (self._odom_topic, self._imu_topic, self._publish_body_tf,
               self._publish_tf, self._standing_height))

    def _tick(self):
        if not self._io.has_state():
            return
        stamp = self.get_clock().now().to_msg()

        # --- IMU (pelvis): always available while powered ---
        imu_raw = self._io.get_imu()
        qw, qx, qy, qz = (float(imu_raw.quaternion[0]), float(imu_raw.quaternion[1]),
                          float(imu_raw.quaternion[2]), float(imu_raw.quaternion[3]))
        gx, gy, gz = (float(imu_raw.gyroscope[0]), float(imu_raw.gyroscope[1]),
                      float(imu_raw.gyroscope[2]))
        ax, ay, az = (float(imu_raw.accelerometer[0]), float(imu_raw.accelerometer[1]),
                      float(imu_raw.accelerometer[2]))

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._imu_frame
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = (
            qx, qy, qz, qw)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gx, gy, gz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.02
        imu.angular_velocity_covariance[0] = 0.001
        imu.angular_velocity_covariance[4] = 0.001
        imu.angular_velocity_covariance[8] = 0.001
        imu.linear_acceleration_covariance[0] = 0.02
        imu.linear_acceleration_covariance[4] = 0.02
        imu.linear_acceleration_covariance[8] = 0.02
        self._imu_pub.publish(imu)

        # --- base_footprint -> base_link (body roll/pitch + standing height) ---
        if self._publish_body_tf:
            roll, pitch = _quat_to_roll_pitch(qw, qx, qy, qz)
            bx, by, bz, bw = _rpy_to_quat(roll, pitch, 0.0)
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._base_frame
            tf.child_frame_id = self._base_link_frame
            tf.transform.translation.z = self._standing_height
            tf.transform.rotation.x, tf.transform.rotation.y = bx, by
            tf.transform.rotation.z, tf.transform.rotation.w = bz, bw
            self._tf.sendTransform(tf)

        # --- /odom_raw (body odometry), published only while fresh ---
        odom_dict = self._io.get_odom()
        if odom_dict is None:
            return
        # Publish /odom_raw only while the body odometry is fresh. The DDS buffer
        # keeps serving the last SportModeState after the stream stops (control-mode
        # dependent), so gate on the age of the last genuinely new sample: past
        # body_odom_timeout, drop /odom_raw and let the EKF fall back to DLIO. A
        # stationary robot still refreshes the stream, so this does not misfire when
        # merely standing still. IMU + body TF keep publishing regardless.
        odom_age = self._io.odom_age()
        if odom_age is not None and odom_age > self._odom_timeout:
            return
        px, py = float(odom_dict["position"][0]), float(odom_dict["position"][1])
        vx, vy = float(odom_dict["velocity"][0]), float(odom_dict["velocity"][1])
        oq = odom_dict["orientation_quaternion"]
        yaw = _quat_to_yaw(float(oq[0]), float(oq[1]), float(oq[2]), float(oq[3]))
        qx2, qy2, qz2, qw2 = _yaw_to_quat(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx2
        odom.pose.pose.orientation.y = qy2
        odom.pose.pose.orientation.z = qz2
        odom.pose.pose.orientation.w = qw2
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = gz  # pelvis gyro z = body yaw rate
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.02
        self._odom_pub.publish(odom)

        # --- odom -> base_footprint, ONLY when no EKF owns it ---
        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = px
            tf.transform.translation.y = py
            tf.transform.rotation.x = qx2
            tf.transform.rotation.y = qy2
            tf.transform.rotation.z = qz2
            tf.transform.rotation.w = qw2
            self._tf.sendTransform(tf)

    def destroy_node(self):
        try:
            self._io.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = G1StateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
