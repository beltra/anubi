#!/usr/bin/env python3
"""
lidar_odom_relay.py

Republishes nav_msgs/Odometry from DLIO with adaptive diagonal covariances.
DLIO outputs all-zero covariances; this relay:
  1. Injects finite base covariances so robot_localization treats DLIO as a
     weighted measurement rather than a hard overwrite.
  2. Scales those covariances up when reliability signals degrade so the EKF
     automatically down-weights DLIO during degenerate conditions:
       - Stale updates (slow scan matching)
       - Velocity twist jumps (proxy for scan glitch)
       - Fast rotation (scan matching degrades)
       - Position jumps (physically impossible displacement over a
         nominal_dt window; windowed so 100 Hz IMU-propagated streams are
         judged at scan cadence, not per message)
     Position jumps bypass EMA so a bad window is immediately seen as
     max-uncertainty; the anchor position is NOT advanced on a jump so
     sustained wrong-position settling keeps triggering the detector.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _diag_cov(values: list) -> list:
    mat = [0.0] * 36
    for i, v in enumerate(values[:6]):
        mat[i * 7] = float(v)
    return mat


def _scale_cov(base: list, scale: float) -> list:
    return [v * scale for v in base]


class LidarOdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odom_relay")

        self.declare_parameter("input_topic", "/lidar_odom")
        self.declare_parameter("output_topic", "/lidar_odom_reliable")
        # Diagonal base covariances [x, y, z, roll, pitch, yaw] (m^2 or rad^2)
        self.declare_parameter("pose_cov", [0.03, 0.03, 0.05, 0.03, 0.03, 0.02])
        # Diagonal base twist covariances [vx, vy, vz, vroll, vpitch, vyaw]
        self.declare_parameter("twist_cov", [0.02, 0.02, 0.05, 0.02, 0.02, 0.02])
        # Reliability tuning
        self.declare_parameter("nominal_dt", 0.10)        # expected DLIO period [s]
        self.declare_parameter("v_jump_threshold", 0.5)   # m/s inter-frame twist spike
        self.declare_parameter("omega_threshold", 0.8)    # rad/s angular speed penalty onset
        # Physical ceiling for position displacement speed. Exceeding this in a
        # single DLIO frame means the displacement is a scan-match artifact, not
        # real motion. Set near the robot's practical navigation speed.
        self.declare_parameter("max_pos_speed", 1.5)      # m/s
        self.declare_parameter("max_scale", 100.0)        # covariance scale ceiling
        # EMA weight for smoothed scale: 0=frozen, 1=no smoothing.
        # At 10 Hz with alpha=0.3, recovery to 1.0 after a 10x spike takes ~1.5 s.
        self.declare_parameter("scale_alpha", 0.3)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._base_pose_cov = _diag_cov(list(self.get_parameter("pose_cov").value))
        self._base_twist_cov = _diag_cov(list(self.get_parameter("twist_cov").value))
        self._nominal_dt = float(self.get_parameter("nominal_dt").value)
        self._v_jump_thr = float(self.get_parameter("v_jump_threshold").value)
        self._omega_thr = float(self.get_parameter("omega_threshold").value)
        self._max_pos_speed = float(self.get_parameter("max_pos_speed").value)
        self._max_scale = float(self.get_parameter("max_scale").value)
        self._alpha = float(self.get_parameter("scale_alpha").value)

        self._last_stamp: float | None = None
        # Position-jump anchor: displacement is evaluated over windows of at
        # least nominal_dt, not per message. DLIO publishes at IMU rate when
        # IMU integration is on (100 Hz on the G1), and a normal cm-scale
        # scan-match correction between 10 ms stamps reads as several m/s --
        # per-frame evaluation flags ~1% of healthy frames as teleports and
        # pins the covariance at max_scale through every turn.
        self._anchor_stamp: float | None = None
        self._anchor_x: float = 0.0
        self._anchor_y: float = 0.0
        self._last_vx: float = 0.0
        self._last_vy: float = 0.0
        self._scale_smooth: float = 1.0
        self._last_warn_t: float = 0.0

        self._pub = self.create_publisher(Odometry, out_topic, qos_profile_sensor_data)
        self.create_subscription(Odometry, in_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"lidar_odom_relay: {in_topic} -> {out_topic} "
            f"(pose_diag={[self._base_pose_cov[i * 7] for i in range(6)]})"
        )

    def _compute_scale(self, msg: Odometry) -> tuple[float, bool]:
        """Return (raw_scale, position_jump_detected).

        position_jump_detected=True means the caller should bypass EMA and
        should NOT update the tracked position so future frames are still
        measured against the last known good position.
        """
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = abs(msg.twist.twist.angular.z)

        scale = 1.0
        pos_jump = False

        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 0.0:
                # Stale update: covariance grows quadratically with the gap so
                # missing one frame (2x nominal_dt) gives 4x covariance.
                scale = max(scale, (dt / self._nominal_dt) ** 2)

                # Velocity twist jump: proxy for scan glitch. Compares reported
                # twist values, so it is publish-rate independent.
                dv = math.hypot(vx - self._last_vx, vy - self._last_vy)
                if dv > self._v_jump_thr:
                    scale = max(scale, dv / self._v_jump_thr)

        # Position jump: displacement speed beyond the robot's physical ceiling
        # is a scan-matching artifact. Evaluated over windows of >= nominal_dt
        # so high-rate IMU-propagated streams are judged at scan cadence, same
        # as a 10 Hz stream. Do NOT advance the anchor on a jump -- sustained
        # wrong-position settling keeps appearing as a jump on later windows.
        if self._anchor_stamp is None:
            self._anchor_stamp = stamp
            self._anchor_x = x
            self._anchor_y = y
        else:
            window_dt = stamp - self._anchor_stamp
            if window_dt >= self._nominal_dt:
                delta_pos = math.hypot(x - self._anchor_x, y - self._anchor_y)
                if delta_pos / window_dt > self._max_pos_speed:
                    pos_jump = True
                    scale = self._max_scale
                else:
                    self._anchor_stamp = stamp
                    self._anchor_x = x
                    self._anchor_y = y
            elif window_dt < 0.0:
                # Time went backwards (bag loop / clock reset): re-anchor.
                self._anchor_stamp = stamp
                self._anchor_x = x
                self._anchor_y = y

        # Fast rotation: scan matching degrades, position estimates less reliable.
        if omega > self._omega_thr:
            scale = max(scale, omega / self._omega_thr)

        self._last_stamp = stamp
        self._last_vx = vx
        self._last_vy = vy

        return min(scale, self._max_scale), pos_jump

    def _cb(self, msg: Odometry) -> None:
        # DLIO republishes scan-match-corrected poses under an already-used
        # header stamp (~17% of messages) plus rare zero stamps. This relay
        # forwards them all: robot_localization on the EKF path resolves
        # same-stamp measurements on its own, and the C8 offline gate shows
        # that de-duplicating HERE (holding a message until a newer stamp
        # arrives) adds a measurement period of latency and thins samples
        # during turns, degrading the EKF (turn yaw p90 2.8 -> 4.2 deg). The
        # non-increasing-stamp TF problem is fixed at odom_to_tf, which is the
        # node that rebroadcasts this stream directly as TF.
        raw, pos_jump = self._compute_scale(msg)

        if pos_jump:
            # Bypass EMA: inject max uncertainty immediately so the EKF ignores
            # this frame entirely and holds position from leg odometry.
            self._scale_smooth = self._max_scale
        else:
            self._scale_smooth = self._alpha * raw + (1.0 - self._alpha) * self._scale_smooth

        scale = self._scale_smooth

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        out.pose.covariance = _scale_cov(self._base_pose_cov, scale)
        out.twist.covariance = _scale_cov(self._base_twist_cov, scale)
        self._pub.publish(out)

        if scale > 2.0:
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if now - self._last_warn_t > 2.0:
                self._last_warn_t = now
                reason = "position jump" if pos_jump else f"scale={scale:.2f}"
                self.get_logger().warning(f"DLIO reliability degraded: {reason}")


def main() -> None:
    rclpy.init()
    node = LidarOdomRelay()
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
