#!/usr/bin/env python3
"""Relay a PointCloud2 topic, rewriting header.stamp onto the local clock.

The Livox Mid-360 stamps cloud headers from its internal timebase, which is
unsynchronized and drifts against the host clock (measured ~20 ms/s, up to
-2.9 s over one G1 run). Every consumer that transforms the cloud at its
stamp (STVL marking, collision_monitor, RViz) then places it with a pose
that old, and the distortion corrector cannot match IMU samples to it at
all once the drift exceeds its timeout. DLIO is unaffected either way: it
reconciles the lidar/IMU timebases internally from per-point timestamps.

This relay rewrites header.stamp = now() - restamp_offset_sec, preserving
the start-of-sweep stamp convention (the driver publishes one sweep-length
after the sweep starts), and shifts the per-point absolute timestamp field
by the same delta so the cloud stays internally consistent: DLIO's deskew
uses per-point-minus-header relative times, which are preserved exactly.
Residual placement error is the transport/processing jitter (~tens of ms)
instead of the unbounded clock drift.
"""

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class CloudRestampRelay(Node):
    def __init__(self) -> None:
        super().__init__("cloud_restamp_relay")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/lidar/points")
        # Sweep duration: the driver publishes when the sweep completes, so
        # now() - sweep_duration approximates the sweep-start stamp the rest
        # of the stack expects.
        self.declare_parameter("restamp_offset_sec", 0.10)
        # How the sweep-start age is derived:
        #   fixed    : restamp_offset_sec (assumes nominal 10 Hz delivery)
        #   interval : EMA of the measured inter-arrival interval
        #              + transport_offset_sec. When the device drops sweeps
        #              (delivery below 10 Hz) the delivered sweep is about one
        #              delivery interval old, which a fixed 0.10 understates.
        #   span     : this cloud's own per-point timestamp span
        #              + transport_offset_sec.
        self.declare_parameter("offset_mode", "fixed")
        self.declare_parameter("transport_offset_sec", 0.0)
        self.declare_parameter("interval_ema_alpha", 0.2)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._offset = Duration(seconds=float(self.get_parameter("restamp_offset_sec").value))
        self._mode = str(self.get_parameter("offset_mode").value)
        if self._mode not in ("fixed", "interval", "span"):
            self.get_logger().warn(f"unknown offset_mode '{self._mode}'; using 'fixed'")
            self._mode = "fixed"
        self._transport_offset = float(self.get_parameter("transport_offset_sec").value)
        self._ema_alpha = float(self.get_parameter("interval_ema_alpha").value)
        self._ema_interval: float | None = None
        self._last_arrival: float | None = None

        # RELIABLE to match the Livox driver's publisher QoS: DLIO subscribes
        # reliable and a best-effort publisher would silently starve it
        # (best-effort subscribers like the corrector still match a reliable
        # publisher). topic_tools relay mirrored the upstream QoS; this does
        # the same explicitly.
        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.create_subscription(PointCloud2, in_topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"cloud_restamp_relay: {in_topic} -> {out_topic} "
            f"(offset_mode={self._mode}, base_offset="
            f"{self._offset.nanoseconds * 1e-9:.3f}s, "
            f"transport_offset={self._transport_offset:.3f}s)"
        )

    def _cb(self, msg: PointCloud2) -> None:
        old_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now = self.get_clock().now()
        msg.header.stamp = (now - self._current_offset(msg, now)).to_msg()
        new_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._shift_point_times(msg, float(new_ns - old_ns))
        self._pub.publish(msg)

    def _current_offset(self, msg: PointCloud2, now) -> Duration:
        """Sweep-start age estimate for this cloud, per offset_mode."""
        if self._mode == "interval":
            arrival = now.nanoseconds * 1e-9
            if self._last_arrival is not None:
                dt = arrival - self._last_arrival
                # Clamp single samples so stalls/bursts cannot poison the EMA.
                dt = min(max(dt, 0.05), 0.50)
                if self._ema_interval is None:
                    self._ema_interval = dt
                else:
                    self._ema_interval += self._ema_alpha * (dt - self._ema_interval)
            self._last_arrival = arrival
            if self._ema_interval is not None:
                return Duration(seconds=self._ema_interval + self._transport_offset)
            return self._offset
        if self._mode == "span":
            span = self._point_time_span_sec(msg)
            if span is not None:
                return Duration(seconds=min(max(span, 0.05), 0.30) + self._transport_offset)
            return self._offset
        return self._offset

    def _point_time_span_sec(self, msg: PointCloud2) -> float | None:
        field = next((f for f in msg.fields if f.name == "timestamp"), None)
        if field is None or field.datatype != PointField.FLOAT64:
            return None
        n = msg.width * msg.height
        if n == 0 or msg.point_step < field.offset + 8:
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        view = np.ndarray(
            shape=(n,), dtype=np.float64, buffer=data,
            offset=field.offset, strides=(msg.point_step,),
        )
        finite = view[np.isfinite(view)]
        if finite.size == 0:
            return None
        # Per-point times are natively non-monotonic (Livox line interleave);
        # the span is max-min, never last-first.
        return float(finite.max() - finite.min()) * 1e-9

    def _shift_point_times(self, msg: PointCloud2, delta_ns: float) -> None:
        """Shift the Livox per-point absolute-ns FLOAT64 'timestamp' field by
        the header delta, keeping per-point-minus-header times unchanged."""
        field = next((f for f in msg.fields if f.name == "timestamp"), None)
        if field is None or field.datatype != PointField.FLOAT64:
            return
        n = msg.width * msg.height
        if n == 0 or msg.point_step < field.offset + 8:
            return
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
        # strided float64 view over the interleaved point buffer
        view = np.ndarray(
            shape=(n,), dtype=np.float64, buffer=data,
            offset=field.offset, strides=(msg.point_step,),
        )
        view += delta_ns
        msg.data = data.tobytes()


def main() -> None:
    rclpy.init()
    node = CloudRestampRelay()
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
