#!/usr/bin/env python3
"""Accumulate motion-compensated non-repetitive LiDAR clouds for 2D scan slicing.

The Unitree L1 emits ~1400 points per frame with a non-repetitive pattern.
After widening the /scan slice to +/-0.30 m (~70% angular coverage per single
frame), one frame is usually enough; this node still concatenates a short
window of recent frames so the remaining ~30% of azimuths get re-confirmed by
the next non-repetitive sub-pattern.

Each buffered frame is transformed into the newest frame's sensor pose through
a fixed reference frame (default `odom`). Without that re-projection, robot
motion during the accumulation window smears walls because older points stay
anchored to the older sensor pose.

Graceful degradation: the newest frame is always publishable without a TF
lookup (it IS the target pose). If TF samples in the fixed_frame are stale or
missing for the older frames, those frames are dropped individually and we
still publish using whatever projected frames succeed -- including just the
newest. This keeps the accumulator's output rate locked to the LiDAR input
rate even when /odom or the fixed_frame TF tree lags.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def strip_to_xyz(msg: PointCloud2) -> PointCloud2:
    """Return a copy of msg containing only x, y, z fields.

    tf2_sensor_msgs.do_transform_cloud in Humble crashes via a dtype assertion
    inside sensor_msgs_py.point_cloud2.create_cloud when the input cloud has
    extra fields (e.g., the Unitree L1's `ring` and `time` fields): its
    `unstructured_to_structured(...)` path produces a structured array whose
    dtype does not match `dtype_from_fields(cloud.fields)`. Stripping to
    xyz-only routes do_transform_cloud through its clean `else` branch.
    """
    field_names = tuple(f.name for f in msg.fields)
    if field_names == ("x", "y", "z"):
        return msg
    pts = read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
    xyz = np.column_stack((pts["x"], pts["y"], pts["z"])).astype(np.float32)
    return create_cloud_xyz32(msg.header, xyz)


def stamp_to_sec(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def fields_signature(msg):
    return tuple(
        (field.name, field.offset, field.datatype, field.count) for field in msg.fields
    )


class CloudAccumulator(Node):
    """Publish one cloud made from the latest frames, each motion-corrected.

    Each buffered frame is transformed from its capture-time sensor pose into
    the newest frame's sensor pose through a fixed reference frame, so the
    concatenated cloud represents the world as seen from the newest pose.
    """

    def __init__(self):
        super().__init__("cloud_accumulator")
        self.declare_parameter("input_topic", "/lidar/points_undistorted")
        self.declare_parameter("output_topic", "/lidar/points_scan_accumulated")
        # Window sized after widening the /scan slice to +/-0.30 m: a single
        # frame already gives ~70% angular coverage, so 3 frames re-confirm
        # the remaining ~30% without adding noticeable latency. The old 0.75 s
        # / 12-frame window was sized for the much sparser +/-0.12 m slice.
        self.declare_parameter("accumulation_window_sec", 0.50)
        self.declare_parameter("max_frames", 3)
        # Fixed frame used for advanced TF lookup: it must be the frame in
        # which the sensor pose at different timestamps is observable (odom).
        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("tf_lookup_timeout_sec", 0.05)
        # Mapping-scan yaw gate: sweeps captured while the body yaws fast are
        # tangentially smeared (the Mid-360 needs ~0.1 s per sweep and the
        # gait adds attitude error), so they thicken map walls along yaw.
        # When > 0, frames whose sensor yaw rate in fixed_frame exceeds this
        # are not published at all; quasi-static sweeps repaint the same
        # walls moments later. 0 disables the gate (safety consumers must
        # keep using the ungated cloud/scan path).
        self.declare_parameter("max_yaw_rate_rad_s", 0.0)
        # What the yaw gate does when it trips:
        #   drop   : publish nothing this cycle (original behavior; /scan
        #            starves to the gated-sweep rate during sustained spins).
        #   single : publish the newest frame alone (no concatenation) and
        #            keep it out of the accumulation buffer, so /scan stays
        #            at input rate while gated sweeps never thicken the
        #            accumulated window.
        self.declare_parameter("yaw_gate_mode", "drop")
        # Input clouds already expressed in fixed_frame (e.g. DLIO's
        # registered scan): frames are world-anchored by the odometry's own
        # registration, so accumulation is plain concatenation with no TF
        # reprojection. The yaw gate then samples sensor_frame instead of the
        # cloud's frame_id.
        self.declare_parameter("input_cloud_in_fixed_frame", False)
        self.declare_parameter("sensor_frame", "lidar_stabilized")
        # World-frame inputs bypass the distortion corrector's body-box
        # self-filter, so remove near-body returns here: drop points within
        # this horizontal radius of sensor_frame's origin. 0 disables.
        self.declare_parameter("self_filter_radius", 0.0)
        # Rear-sector exclusion for the MAPPING scan only: drop points whose
        # azimuth relative to the robot heading lies within this half-width
        # of straight-behind AND whose horizontal range is below the max
        # range. Exists to keep a trailing operator out of the painted map;
        # safety consumers (collision monitor, STVL) must keep using the
        # unfiltered cloud so a person behind the robot still blocks motion.
        # 0 disables. Fixed-frame input mode only.
        self.declare_parameter("exclude_rear_sector_half_width_deg", 0.0)
        self.declare_parameter("exclude_rear_sector_max_range", 1.5)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.window_sec = float(self.get_parameter("accumulation_window_sec").value)
        self.max_frames = int(self.get_parameter("max_frames").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
        )
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rad_s").value)
        self.yaw_gate_mode = str(self.get_parameter("yaw_gate_mode").value)
        if self.yaw_gate_mode not in ("drop", "single"):
            self.get_logger().warn(
                "unknown yaw_gate_mode '%s'; using 'drop'" % self.yaw_gate_mode)
            self.yaw_gate_mode = "drop"
        self.input_in_fixed_frame = bool(
            self.get_parameter("input_cloud_in_fixed_frame").value)
        self.sensor_frame = str(self.get_parameter("sensor_frame").value)
        self.self_filter_radius = float(self.get_parameter("self_filter_radius").value)
        self.rear_sector_half_width = math.radians(float(
            self.get_parameter("exclude_rear_sector_half_width_deg").value))
        self.rear_sector_max_range = float(
            self.get_parameter("exclude_rear_sector_max_range").value)
        self.last_yaw_sample = None  # (tf_stamp_sec, yaw in fixed_frame)
        self.gate_engaged = False

        self.frames = deque()
        self.frame_id = ""
        self.signature = None
        self.point_step = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "cloud_accumulator: %s -> %s (window=%.2fs, max_frames=%d, fixed_frame=%s)"
            % (
                self.input_topic,
                self.output_topic,
                self.window_sec,
                self.max_frames,
                self.fixed_frame,
            )
        )

    def reset_if_layout_changed(self, msg):
        signature = fields_signature(msg)
        layout_changed = (
            self.frame_id
            and (
                msg.header.frame_id != self.frame_id
                or signature != self.signature
                or msg.point_step != self.point_step
            )
        )
        if layout_changed:
            self.frames.clear()
        self.frame_id = msg.header.frame_id
        self.signature = signature
        self.point_step = msg.point_step

    def prune_old_frames(self, newest_stamp):
        while len(self.frames) > self.max_frames:
            self.frames.popleft()
        while self.frames and newest_stamp - self.frames[0][0] > self.window_sec:
            self.frames.popleft()

    def transform_to_newest(self, frame, newest):
        """Re-project frame's points into newest's sensor pose via fixed_frame."""
        if frame is newest:
            return frame
        try:
            tf = self.tf_buffer.lookup_transform_full(
                target_frame=newest.header.frame_id,
                target_time=Time.from_msg(newest.header.stamp),
                source_frame=frame.header.frame_id,
                source_time=Time.from_msg(frame.header.stamp),
                fixed_frame=self.fixed_frame,
                timeout=self.tf_timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            self.get_logger().warn(
                "cloud_accumulator: dropping frame (TF unavailable: %s)" % str(exc),
                throttle_duration_sec=2.0,
            )
            return None
        # Frames have already been stripped to xyz-only in cloud_callback,
        # so do_transform_cloud takes the clean path that handles the
        # transformation correctly. See strip_to_xyz docstring for context.
        transformed = do_transform_cloud(frame, tf)
        transformed.header.stamp = newest.header.stamp
        transformed.header.frame_id = newest.header.frame_id
        return transformed

    def yaw_rate_exceeded(self, msg, stamp_sec):
        """True when the sensor yaw rate in fixed_frame exceeds the gate.

        The rate is sampled from the LATEST available transform, not the
        frame's exact stamp: the odom TF for this cloud is published by the
        odometry at the same instant the cloud arrives, so an exact-stamp
        lookup races it and a lookup failure would silently disable the gate
        during exactly the fast-turn frames it exists for. A latest-sample
        estimate lags one TF period at most, which the hysteresis below
        absorbs at turn onset.
        """
        if self.max_yaw_rate <= 0.0:
            return False
        gate_frame = (
            self.sensor_frame if self.input_in_fixed_frame else msg.header.frame_id
        )
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                gate_frame,
                Time(),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            # No pose at all yet: let the frame through rather than starving
            # consumers during TF startup gaps.
            return False
        tf_sec = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        prev = self.last_yaw_sample
        if prev is None or tf_sec > prev[0]:
            self.last_yaw_sample = (tf_sec, yaw)
        if prev is None or tf_sec <= prev[0]:
            return self.gate_engaged
        dyaw = (yaw - prev[1] + math.pi) % (2.0 * math.pi) - math.pi
        rate = abs(dyaw) / (tf_sec - prev[0])
        # Hysteresis: once gated, stay gated until the rate falls below half
        # the threshold, so turn-onset/exit boundary sweeps are dropped too.
        threshold = self.max_yaw_rate * (0.5 if self.gate_engaged else 1.0)
        self.gate_engaged = rate > threshold
        return self.gate_engaged

    def filter_near_body(self, frames, newest):
        """Drop near-body returns and, when enabled, the rear sector.

        Used only in fixed-frame input mode, where the registered cloud has
        not passed the distortion corrector's body-box self-filter. Both
        tests run against the sensor pose at each frame's own stamp so body
        returns and the rear sector stay anchored to the robot while walking.
        sensor_frame is gravity-levelled, so its yaw is the body heading.
        """
        out = []
        for frame in frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    self.sensor_frame,
                    Time.from_msg(frame.header.stamp),
                    timeout=self.tf_timeout,
                )
                ox = tf.transform.translation.x
                oy = tf.transform.translation.y
                q = tf.transform.rotation
                heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.TransformException,
            ):
                out.append(frame)
                continue
            pts = read_points(frame, field_names=("x", "y", "z"), skip_nans=False)
            xyz = np.column_stack((pts["x"], pts["y"], pts["z"])).astype(np.float32)
            dx = xyz[:, 0] - ox
            dy = xyz[:, 1] - oy
            rng = np.hypot(dx, dy)
            keep = rng > self.self_filter_radius
            if self.rear_sector_half_width > 0.0:
                azimuth = np.arctan2(dy, dx) - heading
                # angular distance from straight-behind (pi), wrapped
                from_rear = np.abs((azimuth % (2.0 * math.pi)) - math.pi)
                in_sector = (from_rear <= self.rear_sector_half_width) & (
                    rng <= self.rear_sector_max_range)
                keep &= ~in_sector
            if not keep.any():
                continue
            out.append(create_cloud_xyz32(frame.header, xyz[keep]))
        return out

    def cloud_callback(self, msg):
        if msg.point_step == 0 or not msg.data:
            return

        if self.yaw_rate_exceeded(msg, stamp_to_sec(msg)):
            if self.yaw_gate_mode == "single":
                # Keep /scan alive during sustained spins: emit this frame
                # alone (it is GICP-registered, not garbage) but keep it out
                # of the accumulation buffer so gated sweeps never thicken
                # the accumulated window.
                single = strip_to_xyz(msg)
                frames = [single]
                if self.input_in_fixed_frame and (
                        self.self_filter_radius > 0.0
                        or self.rear_sector_half_width > 0.0):
                    frames = self.filter_near_body(frames, single)
                if frames:
                    self.publish_frames(frames, single)
            return

        # Reduce to xyz-only up front so every frame in the buffer has the
        # same lean layout and do_transform_cloud avoids its dtype bug on
        # clouds with extra fields. Downstream pointcloud_to_laserscan only
        # consumes x, y, z.
        msg = strip_to_xyz(msg)

        self.reset_if_layout_changed(msg)
        stamp_sec = stamp_to_sec(msg)
        self.frames.append((stamp_sec, msg))
        self.prune_old_frames(stamp_sec)

        if not self.frames:
            return

        newest = self.frames[-1][1]
        if self.input_in_fixed_frame:
            # World-anchored frames: the odometry already registered each
            # frame in fixed_frame, so the window is a plain concatenation.
            transformed_frames = [f for _, f in self.frames]
        else:
            transformed_frames = []
            for _, frame in self.frames:
                projected = self.transform_to_newest(frame, newest)
                if projected is not None:
                    transformed_frames.append(projected)

        if not transformed_frames:
            return

        if self.input_in_fixed_frame and (
                self.self_filter_radius > 0.0 or self.rear_sector_half_width > 0.0):
            transformed_frames = self.filter_near_body(transformed_frames, newest)
            if not transformed_frames:
                return

        self.publish_frames(transformed_frames, newest)

    def publish_frames(self, frames, newest):
        point_count = sum(len(f.data) // f.point_step for f in frames)
        if point_count == 0:
            return

        out = PointCloud2()
        out.header = newest.header
        out.height = 1
        out.width = point_count
        out.fields = newest.fields
        out.is_bigendian = newest.is_bigendian
        out.point_step = newest.point_step
        out.row_step = out.point_step * out.width
        out.is_dense = all(f.is_dense for f in frames)
        out.data = b"".join(bytes(f.data) for f in frames)

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudAccumulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
