#!/usr/bin/env python3
"""
lowstate_joint_state_publisher.py

Subscribe to /lowstate (unitree_go/LowState) and publish /joint_states
(sensor_msgs/JointState) at a fixed rate. Used for offline bag replay where
go2_hw_bridge is intentionally absent (no Sport API / watchdog / TF
publishing).

Motor index -> joint name mapping mirrors go2_hw_bridge.cpp and is grounded
in unitree_sdk2/include/unitree/dds_wrapper/robots/go2/defines.h
(FR=0..2, FL=3..5, RR=6..8, RL=9..11 with 0=hip, 1=thigh, 2=calf).
Joint names match leg.xacro in go2_description.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


JOINT_NAMES = (
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",  # FR (0..2)
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",  # FL (3..5)
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",  # RR (6..8)
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",  # RL (9..11)
)


class LowStateJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("lowstate_joint_state_publisher")

        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_rate", 50.0)

        lowstate_topic = self.get_parameter("lowstate_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        rate = float(self.get_parameter("publish_rate").value)

        self._latest: LowState | None = None

        self._sub = self.create_subscription(
            LowState, lowstate_topic, self._on_low_state, 10
        )
        self._pub = self.create_publisher(JointState, joint_states_topic, 10)

        period = 1.0 / rate if rate > 0.0 else 0.02
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"lowstate_joint_state_publisher: {lowstate_topic} -> "
            f"{joint_states_topic} @ {rate:.1f} Hz"
        )

    def _on_low_state(self, msg: LowState) -> None:
        self._latest = msg

    def _publish(self) -> None:
        if self._latest is None:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(self._latest.motor_state[i].q) for i in range(12)]
        js.velocity = [float(self._latest.motor_state[i].dq) for i in range(12)]
        js.effort = [float(self._latest.motor_state[i].tau_est) for i in range(12)]
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = LowStateJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
