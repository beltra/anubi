#!/usr/bin/env python3
"""Low-level A2-W controller for the articulated Gazebo model (locomotion:=policy).

/cmd_vel_safe -> 12 leg position targets (Fortress JointPositionController, force PD)
              -> 4 wheel velocity targets (Fortress JointController, force mode)

Two modes after a stand-up phase that folds the legs from the zero-angle spawn pose:
  policy : Isaac Lab / robot_lab actor (training/a2w_robot_lab), run in numpy at 50 Hz.
           Observation and action layout must match the training task exactly:
           obs = [ang_vel*0.25, projected_gravity, cmd(vx, vy, wz), leg+wheel joint_pos_rel
                  (wheel entries zeroed), joint_vel*0.05, last_action]
           act = [12 leg position offsets (hip*0.125, thigh/calf*0.25),
                  4 wheel velocity targets (*5.0 rad/s)]
  stance : no policy file. Legs hold the standing pose and the wheels skid-steer
           (vx, wz); vy is ignored because the wheel axles cannot steer.
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

STATE_WAIT = "wait"
STATE_STAND = "stand"
STATE_RUN = "run"
STATE_FALLEN = "fallen"


def quat_rotate_inverse(qx, qy, qz, qw, v):
    """Rotate world vector v into the body frame of orientation q (x, y, z, w)."""
    q_vec = np.array([qx, qy, qz])
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(q_vec, v) * qw * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


class MlpActor:
    """ELU MLP from export_policy_npz.py: keys W0, b0, W1, b1, ... (last layer linear)."""

    def __init__(self, path):
        data = np.load(path, allow_pickle=False)
        self.layers = []
        i = 0
        while "W%d" % i in data:
            self.layers.append((data["W%d" % i].astype(np.float64), data["b%d" % i].astype(np.float64)))
            i += 1
        if not self.layers:
            raise ValueError("no W0/b0 arrays in " + path)
        self.metadata = json.loads(str(data["metadata"])) if "metadata" in data else {}
        self.obs_dim = self.layers[0][0].shape[1]
        self.act_dim = self.layers[-1][0].shape[0]

    def __call__(self, obs):
        x = obs
        for idx, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if idx < len(self.layers) - 1:
                x = np.where(x > 0.0, x, np.expm1(x))
        return x


class A2WPolicyController(Node):
    def __init__(self):
        super().__init__("a2w_policy_controller")
        self.robot_name = self.declare_parameter("robot_name", "a2w").value
        self.policy_path = self.declare_parameter("policy_path", "").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe").value
        self.leg_joints = list(self.declare_parameter("leg_joints", [""]).value)
        self.wheel_joints = list(self.declare_parameter("wheel_joints", [""]).value)
        self.control_period = self.declare_parameter("control_period_sec", 0.02).value
        self.stand_duration = self.declare_parameter("stand_duration_sec", 2.0).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout_sec", 0.5).value
        self.stand_hip = self.declare_parameter("stand_hip", 0.0).value
        self.stand_thigh = self.declare_parameter("stand_thigh", 0.8).value
        self.stand_calf = self.declare_parameter("stand_calf", -1.5).value
        self.hip_action_scale = self.declare_parameter("hip_action_scale", 0.125).value
        self.leg_action_scale = self.declare_parameter("leg_action_scale", 0.25).value
        self.wheel_action_scale = self.declare_parameter("wheel_action_scale", 5.0).value
        self.ang_vel_scale = self.declare_parameter("ang_vel_scale", 0.25).value
        self.joint_vel_scale = self.declare_parameter("joint_vel_scale", 0.05).value
        self.cmd_limits = list(self.declare_parameter("cmd_limits", [1.0, 1.0, 1.0]).value)
        # Stance mode: wheel contact y = hip 0.075 + thigh 0.128 + wheel 0.06 (a2w.urdf.xacro).
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.095).value
        self.wheel_track = self.declare_parameter("wheel_track", 0.526).value
        self.skid_track_scale = self.declare_parameter("skid_track_scale", 1.0).value
        self.max_wheel_speed = self.declare_parameter("max_wheel_speed", 30.0).value
        self.fallen_gravity_z = self.declare_parameter("fallen_gravity_z", -0.5).value

        if len(self.leg_joints) != 12 or len(self.wheel_joints) != 4:
            raise ValueError("expected 12 leg_joints and 4 wheel_joints")
        self.joint_names = self.leg_joints + self.wheel_joints
        self.default_leg_pos = np.array([self._stand_angle(j) for j in self.leg_joints])
        self.leg_action_scales = np.array(
            [self.hip_action_scale if "_hip_" in j else self.leg_action_scale for j in self.leg_joints])

        self.actor = None
        if self.policy_path:
            self.actor = MlpActor(self.policy_path)
            self._check_policy_metadata()
            expected_obs = 9 + 3 * len(self.joint_names)
            if self.actor.obs_dim != expected_obs or self.actor.act_dim != len(self.joint_names):
                raise ValueError(
                    "policy dims obs=%d act=%d, expected obs=%d act=%d"
                    % (self.actor.obs_dim, self.actor.act_dim, expected_obs, len(self.joint_names)))
        self.get_logger().info(
            "A2-W low-level controller: %s mode"
            % ("policy (" + self.policy_path + ")" if self.actor else "stance + skid-steer"))

        self.joint_pos = None
        self.joint_vel = None
        self.imu = None
        self.cmd = np.zeros(3)
        self.last_cmd_time = None
        self.last_action = np.zeros(len(self.joint_names))
        self.state = STATE_WAIT
        self.stand_start_time = None
        self.stand_start_pos = None

        prefix = "/model/%s/joint/" % self.robot_name
        self.leg_pubs = [self.create_publisher(Float64, prefix + j + "/cmd_pos", 10) for j in self.leg_joints]
        self.wheel_pubs = [self.create_publisher(Float64, prefix + j + "/cmd_vel", 10) for j in self.wheel_joints]
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd, 10)
        self.create_timer(self.control_period, self.on_timer)

    def _stand_angle(self, joint):
        if "_hip_" in joint:
            return self.stand_hip
        if "_thigh_" in joint:
            return self.stand_thigh
        return self.stand_calf

    def _check_policy_metadata(self):
        meta = self.actor.metadata
        for key, expected in (("leg_joints", self.leg_joints), ("wheel_joints", self.wheel_joints)):
            if key in meta and list(meta[key]) != expected:
                raise ValueError("policy %s %s != controller %s" % (key, meta[key], expected))

    def on_joint_states(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if any(j not in index for j in self.joint_names):
            return
        self.joint_pos = np.array([msg.position[index[j]] for j in self.joint_names])
        if len(msg.velocity) == len(msg.name):
            self.joint_vel = np.array([msg.velocity[index[j]] for j in self.joint_names])
        else:
            self.joint_vel = np.zeros(len(self.joint_names))

    def on_imu(self, msg):
        self.imu = msg

    def on_cmd(self, msg):
        limits = self.cmd_limits
        self.cmd = np.array([
            float(np.clip(msg.linear.x, -limits[0], limits[0])),
            float(np.clip(msg.linear.y, -limits[1], limits[1])),
            float(np.clip(msg.angular.z, -limits[2], limits[2])),
        ])
        self.last_cmd_time = self.get_clock().now()

    def current_cmd(self):
        if self.last_cmd_time is None:
            return np.zeros(3)
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return self.cmd if age <= self.cmd_timeout else np.zeros(3)

    def projected_gravity(self):
        q = self.imu.orientation
        return quat_rotate_inverse(q.x, q.y, q.z, q.w, np.array([0.0, 0.0, -1.0]))

    def publish(self, leg_targets, wheel_targets):
        for pub, value in zip(self.leg_pubs, leg_targets):
            pub.publish(Float64(data=float(value)))
        for pub, value in zip(self.wheel_pubs, wheel_targets):
            pub.publish(Float64(data=float(value)))

    def on_timer(self):
        if self.joint_pos is None or self.imu is None:
            return
        now = self.get_clock().now()

        if self.state == STATE_WAIT:
            self.state = STATE_STAND
            self.stand_start_time = now
            self.stand_start_pos = self.joint_pos[:12].copy()
            self.get_logger().info("Standing up over %.1f s" % self.stand_duration)

        if self.state == STATE_STAND:
            elapsed = (now - self.stand_start_time).nanoseconds * 1e-9
            alpha = min(1.0, elapsed / max(1e-3, self.stand_duration))
            alpha = 0.5 - 0.5 * math.cos(math.pi * alpha)
            legs = (1.0 - alpha) * self.stand_start_pos + alpha * self.default_leg_pos
            self.publish(legs, np.zeros(4))
            if elapsed >= self.stand_duration + 0.5:
                self.state = STATE_RUN
                self.get_logger().info("Stance reached, tracking %s" % self.cmd_vel_topic)
            return

        gravity = self.projected_gravity()
        if gravity[2] > self.fallen_gravity_z:
            if self.state != STATE_FALLEN:
                self.get_logger().error("A2-W fell over (projected gravity z %.2f); wheels stopped" % gravity[2])
                self.state = STATE_FALLEN
            self.publish(self.default_leg_pos, np.zeros(4))
            return
        if self.state == STATE_FALLEN:
            self.get_logger().info("A2-W upright again, resuming")
            self.state = STATE_RUN

        cmd = self.current_cmd()
        if self.actor is None:
            self.run_stance(cmd)
        else:
            self.run_policy(cmd, gravity)

    def run_stance(self, cmd):
        half_track = 0.5 * self.wheel_track * self.skid_track_scale
        v_left = cmd[0] - cmd[2] * half_track
        v_right = cmd[0] + cmd[2] * half_track
        wheel = []
        for joint in self.wheel_joints:
            v = v_left if joint.startswith(("FL", "RL")) else v_right
            wheel.append(float(np.clip(v / self.wheel_radius, -self.max_wheel_speed, self.max_wheel_speed)))
        self.publish(self.default_leg_pos, wheel)

    def run_policy(self, cmd, gravity):
        w = self.imu.angular_velocity
        joint_pos_rel = self.joint_pos.copy()
        joint_pos_rel[:12] -= self.default_leg_pos
        joint_pos_rel[12:] = 0.0
        obs = np.concatenate([
            np.array([w.x, w.y, w.z]) * self.ang_vel_scale,
            gravity,
            cmd,
            joint_pos_rel,
            self.joint_vel * self.joint_vel_scale,
            self.last_action,
        ])
        action = np.clip(self.actor(np.clip(obs, -100.0, 100.0)), -100.0, 100.0)
        self.last_action = action
        legs = self.default_leg_pos + self.leg_action_scales * action[:12]
        wheels = self.wheel_action_scale * action[12:]
        self.publish(legs, wheels)


def main():
    rclpy.init()
    node = A2WPolicyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
