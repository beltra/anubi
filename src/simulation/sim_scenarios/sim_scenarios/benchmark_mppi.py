#!/usr/bin/env python3
"""
benchmark_mppi.py
Run repeatable Nav2 benchmarks and detect in-place spin episodes from odometry.

This runner focuses on action-level measurements:
- ComputePathToPose latency per goal
- NavigateToPose completion and elapsed time per goal
- Spin-in-place episodes while navigation is active

Spin episode definition (from MPPI plan):
- duration > 0.75 s
- planar_speed < 0.05 m/s
- abs(yaw_rate) > 0.35 rad/s

Usage example:
  ros2 run sim_scenarios benchmark_mppi \
    --ros-args -p goals_file:=benchmark_goals_default_safe.yaml -p scenario:=crowded_room
"""

import json
import math
import os
import time
from action_msgs.msg import GoalStatus
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
import yaml


@dataclass
class GoalResult:
    scenario: str
    goal_index: int
    goal_name: str
    compute_path_ok: bool
    compute_path_latency_s: float
    navigate_ok: bool
    navigate_latency_s: float
    spin_episodes: int


class MppiBenchmark(Node):

    def __init__(self) -> None:
        super().__init__("benchmark_mppi")

        self.declare_parameter("goals_file", "benchmark_goals_default_safe.yaml")
        self.declare_parameter("scenario", "crowded_room")
        self.declare_parameter("report_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("goal_timeout_s", 180.0)
        self.declare_parameter("enable_pose_reset", True)
        self.declare_parameter("world_name", "indoor_office")
        self.declare_parameter("robot_entity_name", "go2")
        self.declare_parameter("robot_start_z", 0.32)
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("settle_time_s", 1.5)

        self._goals_file = str(self.get_parameter("goals_file").value)
        self._scenario = str(self.get_parameter("scenario").value)
        self._report_file = str(self.get_parameter("report_file").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)
        self._enable_pose_reset = bool(self.get_parameter("enable_pose_reset").value)
        self._world_name = str(self.get_parameter("world_name").value)
        self._robot_entity_name = str(self.get_parameter("robot_entity_name").value)
        self._robot_start_z = float(self.get_parameter("robot_start_z").value)
        self._initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self._settle_time_s = float(self.get_parameter("settle_time_s").value)

        self._compute_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._set_pose_client = self.create_client(
            SetEntityPose, f"/world/{self._world_name}/set_pose"
        )

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._initialpose_topic, 10
        )

        self._spin_active = False
        self._spin_start_s = 0.0
        self._spin_episodes = 0
        self._nav_running = False

    def _resolve_goals_file(self, path_or_name: str) -> str:
        if os.path.isabs(path_or_name):
            return path_or_name
        name = path_or_name if path_or_name.endswith(".yaml") else path_or_name + ".yaml"
        share = get_package_share_directory("sim_scenarios")
        return os.path.join(share, "scenarios", name)

    def _odom_cb(self, msg: Odometry) -> None:
        if not self._nav_running:
            self._spin_active = False
            return

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        planar_speed = math.hypot(vx, vy)

        now_s = self.get_clock().now().nanoseconds / 1e9
        in_spin_window = planar_speed < 0.05 and abs(wz) > 0.35

        if in_spin_window and not self._spin_active:
            self._spin_active = True
            self._spin_start_s = now_s
            return

        if not in_spin_window and self._spin_active:
            duration = now_s - self._spin_start_s
            if duration > 0.75:
                self._spin_episodes += 1
            self._spin_active = False

    def _load_manifest(self) -> Dict:
        resolved = self._resolve_goals_file(self._goals_file)
        if not os.path.isfile(resolved):
            raise FileNotFoundError(f"Goals file not found: {resolved}")
        with open(resolved, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _wait_servers(self) -> None:
        self.get_logger().info("Waiting for Nav2 action servers...")
        if not self._compute_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("Action server compute_path_to_pose not available")
        if not self._navigate_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("Action server navigate_to_pose not available")
        if self._enable_pose_reset:
            if not self._set_pose_client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(
                    f"Service {self._set_pose_client.srv_name} not available. "
                    "Set enable_pose_reset:=false or verify Gazebo world is running."
                )

    def _publish_initialpose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.09

        # Publish a short burst to improve delivery to localization consumers.
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._initialpose_pub.publish(msg)
            time.sleep(0.05)

    def _reset_start_pose(self, start_pose: Dict) -> None:
        x = float(start_pose["x"])
        y = float(start_pose["y"])
        yaw = float(start_pose.get("yaw", 0.0))
        z = float(start_pose.get("z", self._robot_start_z))

        req = SetEntityPose.Request()
        req.entity.name = self._robot_entity_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._set_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                "Failed to reset robot pose via Gazebo set_pose service. "
                f"Check robot_entity_name parameter (current: {self._robot_entity_name})."
            )

        self._publish_initialpose(x, y, yaw)
        if self._settle_time_s > 0.0:
            time.sleep(self._settle_time_s)

    def _compute_path_latency(self, target: PoseStamped) -> (bool, float):
        goal = ComputePathToPose.Goal()
        goal.goal = target

        start = time.monotonic()
        send_future = self._compute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, time.monotonic() - start

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self._goal_timeout_s)
        latency = time.monotonic() - start

        result_wrapped = result_future.result()
        if result_wrapped is None:
            return False, latency
        return int(result_wrapped.status) == GoalStatus.STATUS_SUCCEEDED, latency

    def _navigate_and_measure(self, target: PoseStamped) -> (bool, float, int):
        goal = NavigateToPose.Goal()
        goal.pose = target

        self._spin_episodes = 0
        self._spin_active = False
        self._nav_running = True

        start = time.monotonic()
        send_future = self._navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._nav_running = False
            return False, time.monotonic() - start, 0

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self._goal_timeout_s)

        self._nav_running = False
        if self._spin_active:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self._spin_start_s > 0.75:
                self._spin_episodes += 1
            self._spin_active = False

        latency = time.monotonic() - start
        result_wrapped = result_future.result()
        if result_wrapped is None:
            return False, latency, self._spin_episodes

        status_ok = int(result_wrapped.status) == 4
        return status_ok, latency, self._spin_episodes

    def run(self) -> int:
        self._wait_servers()
        manifest = self._load_manifest()

        scenarios: List[Dict] = manifest.get("scenarios", [])
        selected = next((s for s in scenarios if s.get("name") == self._scenario), None)
        if selected is None:
            raise RuntimeError(f"Scenario '{self._scenario}' not found in manifest")

        goals = selected.get("goals", [])
        if not goals:
            raise RuntimeError(f"Scenario '{self._scenario}' has no goals")

        start_pose = selected.get("start_pose")
        if self._enable_pose_reset:
            if not isinstance(start_pose, dict):
                raise RuntimeError(
                    f"Scenario '{self._scenario}' has no start_pose in manifest"
                )
            self.get_logger().info(
                f"Resetting robot pose for scenario '{self._scenario}' "
                f"to x={start_pose.get('x')}, y={start_pose.get('y')}, yaw={start_pose.get('yaw', 0.0)}"
            )
            self._reset_start_pose(start_pose)

        self.get_logger().info(
            f"Running benchmark for scenario '{self._scenario}' with {len(goals)} goals"
        )

        rows: List[GoalResult] = []
        for i, g in enumerate(goals, start=1):
            name = str(g.get("name", f"goal_{i}"))
            target = self._pose_stamped(g["x"], g["y"], g.get("yaw", 0.0))

            cp_ok, cp_dt = self._compute_path_latency(target)
            nav_ok, nav_dt, spin_count = self._navigate_and_measure(target)

            row = GoalResult(
                scenario=self._scenario,
                goal_index=i,
                goal_name=name,
                compute_path_ok=cp_ok,
                compute_path_latency_s=round(cp_dt, 3),
                navigate_ok=nav_ok,
                navigate_latency_s=round(nav_dt, 3),
                spin_episodes=spin_count,
            )
            rows.append(row)
            self.get_logger().info(
                f"[{i}/{len(goals)}] {name}: path_ok={cp_ok} nav_ok={nav_ok} "
                f"path_dt={row.compute_path_latency_s}s nav_dt={row.navigate_latency_s}s spins={spin_count}"
            )

        payload = {
            "generated_at_unix": time.time(),
            "scenario": self._scenario,
            "frame_id": self._frame_id,
            "goal_timeout_s": self._goal_timeout_s,
            "results": [asdict(r) for r in rows],
            "summary": {
                "goals": len(rows),
                "compute_path_success": sum(1 for r in rows if r.compute_path_ok),
                "navigate_success": sum(1 for r in rows if r.navigate_ok),
                "spin_episodes_total": sum(r.spin_episodes for r in rows),
                "compute_path_max_s": max((r.compute_path_latency_s for r in rows), default=0.0),
            },
        }

        out = self._report_file.strip()
        if not out:
            stamp = int(time.time())
            out = f"/tmp/mppi_benchmark_{self._scenario}_{stamp}.json"

        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        self.get_logger().info(f"Benchmark report written to: {out}")
        return 0


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MppiBenchmark()
    code = 1
    try:
        code = node.run()
    except Exception as exc:  # pragma: no cover
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if code != 0:
        raise SystemExit(code)


if __name__ == "__main__":
    main()
