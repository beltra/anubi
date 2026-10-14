#!/usr/bin/env python3
"""Sim-to-sim check: run an exported A2-W policy (.npz) in MuJoCo and record a video.

Loads the same URDF the policy was trained on (unitree_a2w/description/urdf/a2w.urdf),
adds a floating base, floor and torque actuators, and reproduces the training actuator
model in Python: leg PD (kp 100, kd 3, 120/120/180 Nm) and wheel velocity damping
(kd 1.5, 24.5 Nm) at a 5 ms physics step with the policy at 50 Hz (decimation 4).
A scripted command sequence drives the robot; the camera tracks the base.

    MUJOCO_GL=egl python mujoco_play.py a2w_flat.npz a2w_drive.mp4
"""

import argparse
import json
import os

import imageio.v2 as imageio
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "unitree_a2w", "description", "urdf", "a2w.urdf")
MESH_DIR = os.path.join(HERE, "unitree_a2w", "description", "meshes")

LEG_JOINTS = [leg + "_" + part + "_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
WHEEL_JOINTS = [leg + "_wheel_joint" for leg in ("FR", "FL", "RR", "RL")]
STAND = {"hip": 0.0, "thigh": 0.8, "calf": -1.5}
LEG_KP, LEG_KD = 100.0, 3.0
WHEEL_KD, WHEEL_TAU = 1.5, 24.5
SIM_DT, DECIMATION = 0.005, 4

# (duration s, vx, vy, wz)
COMMANDS = [
    (2.0, 0.0, 0.0, 0.0),
    (4.0, 1.0, 0.0, 0.0),
    (3.0, 0.6, 0.0, 0.8),
    (3.0, 0.0, 0.0, -1.0),
    (3.0, 0.0, 0.5, 0.0),
    (3.0, -0.5, 0.0, 0.0),
    (2.0, 0.0, 0.0, 0.0),
]


def build_model():
    # Keep the URDF visual meshes (MuJoCo drops them by default); they are made
    # non-colliding below so contact uses the same primitives as Isaac Lab.
    with open(URDF) as fh:
        urdf = fh.read()
    urdf = urdf.replace(
        '<robot name="a2w">',
        '<robot name="a2w"><mujoco><compiler discardvisual="false" fusestatic="true"/></mujoco>', 1)
    spec = mujoco.MjSpec.from_string(urdf)
    spec.meshdir = MESH_DIR
    for mesh in spec.meshes:
        mesh.file = os.path.basename(mesh.file)
    spec.option.timestep = SIM_DT
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.55, 0.55, 0.55]
    for geom in spec.geoms:
        if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
            geom.contype, geom.conaffinity, geom.group = 0, 0, 1
            geom.rgba = [0.80, 0.82, 0.90, 1.0]
        else:
            geom.group = 3
            geom.friction = [1.0, 0.005, 0.0001]
    base = spec.body("base_link")
    base.add_freejoint()
    world = spec.worldbody
    world.add_light(type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, pos=[0, 0, 5], dir=[0.3, 0.3, -1], diffuse=[0.6, 0.6, 0.6], castshadow=True)
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.55, 0.6, 0.65], rgb2=[0.42, 0.47, 0.52], width=512, height=512)
    mat = spec.add_material(name="grid")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    mat.texrepeat = [40, 40]
    world.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[50, 50, 0.1], material="grid",
                   friction=[1.0, 0.005, 0.0001])
    for name in LEG_JOINTS + WHEEL_JOINTS:
        limit = 180.0 if "calf" in name else (WHEEL_TAU if "wheel" in name else 120.0)
        spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          ctrllimited=True, ctrlrange=[-limit, limit])
    model = spec.compile()
    return model


class Actor:
    def __init__(self, path):
        data = np.load(path, allow_pickle=False)
        self.layers = []
        i = 0
        while "W%d" % i in data:
            self.layers.append((data["W%d" % i].astype(np.float64), data["b%d" % i].astype(np.float64)))
            i += 1
        self.meta = json.loads(str(data["metadata"])) if "metadata" in data else {}

    def __call__(self, x):
        for idx, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if idx < len(self.layers) - 1:
                x = np.where(x > 0.0, x, np.expm1(x))
        return x


def quat_rotate_inverse(q_wxyz, v):
    w, x, y, z = q_wxyz
    q_vec = np.array([x, y, z])
    return v * (2.0 * w * w - 1.0) - np.cross(q_vec, v) * w * 2.0 + q_vec * np.dot(q_vec, v) * 2.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("policy")
    parser.add_argument("video")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=25)
    args = parser.parse_args()

    model = build_model()
    data = mujoco.MjData(model)
    actor = Actor(args.policy)

    jnt = [model.joint(n) for n in LEG_JOINTS + WHEEL_JOINTS]
    qadr = np.array([j.qposadr[0] for j in jnt])
    vadr = np.array([j.dofadr[0] for j in jnt])
    act_ids = np.array([model.actuator(n).id for n in LEG_JOINTS + WHEEL_JOINTS])
    default_leg = np.array([STAND[n.split("_")[1]] for n in LEG_JOINTS])
    leg_scale = np.array([0.125 if "_hip_" in n else 0.25 for n in LEG_JOINTS])
    leg_limit = np.array([180.0 if "calf" in n else 120.0 for n in LEG_JOINTS])

    data.qpos[0:3] = [0.0, 0.0, 0.53]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[qadr[:12]] = default_leg
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = model.body("base_link").id
    cam.distance, cam.elevation, cam.azimuth = 3.2, -20.0, 135.0

    writer = imageio.get_writer(args.video, fps=args.fps, codec="libx264", quality=8)
    last_action = np.zeros(16)
    leg_target, wheel_target = default_leg.copy(), np.zeros(4)
    policy_dt = SIM_DT * DECIMATION
    frame_every = max(1, int(round(1.0 / (args.fps * policy_dt))))
    step, log = 0, []
    summary = []
    for duration, vx, vy, wz in COMMANDS:
        cmd = np.array([vx, vy, wz])
        seg = []
        n_steps = int(round(duration / policy_dt))
        for k in range(n_steps):
            q = data.qpos[qadr]
            dq = data.qvel[vadr]
            ang_vel = data.qvel[3:6].copy()  # free joint angular velocity is in the body frame
            gravity = quat_rotate_inverse(data.qpos[3:7], np.array([0.0, 0.0, -1.0]))
            joint_pos_rel = q.copy()
            joint_pos_rel[:12] -= default_leg
            joint_pos_rel[12:] = 0.0
            obs = np.concatenate([ang_vel * 0.25, gravity, cmd, joint_pos_rel, dq * 0.05, last_action])
            action = np.clip(actor(np.clip(obs, -100, 100)), -100, 100)
            last_action = action
            leg_target = default_leg + leg_scale * action[:12]
            wheel_target = 5.0 * action[12:]
            for _ in range(DECIMATION):
                q = data.qpos[qadr]
                dq = data.qvel[vadr]
                tau_leg = np.clip(LEG_KP * (leg_target - q[:12]) - LEG_KD * dq[:12], -leg_limit, leg_limit)
                tau_wheel = np.clip(WHEEL_KD * (wheel_target - dq[12:]), -WHEEL_TAU, WHEEL_TAU)
                data.ctrl[act_ids] = np.concatenate([tau_leg, tau_wheel])
                mujoco.mj_step(model, data)
            if step % frame_every == 0:
                renderer.update_scene(data, camera=cam)
                writer.append_data(renderer.render())
            if k >= int(1.0 / policy_dt):  # skip the first second of each segment (transient)
                vel_body = quat_rotate_inverse(data.qpos[3:7], data.qvel[0:3])
                seg.append([vel_body[0], vel_body[1], data.qvel[5]])
            if step % 25 == 0:
                vel_body = quat_rotate_inverse(data.qpos[3:7], data.qvel[0:3])
                log.append((step * policy_dt, cmd.tolist(), vel_body[:2].round(2).tolist(),
                            round(float(data.qvel[5]), 2), round(float(data.qpos[2]), 3), gravity[2].round(2)))
            step += 1
        mean = np.mean(seg, axis=0) if seg else np.zeros(3)
        summary.append((cmd, mean))
    writer.close()
    for t, c, v, w, z, g in log:
        print("t=%5.1f cmd=%s vel_xy=%s wz=%5.2f base_z=%.3f grav_z=%5.2f" % (t, c, v, w, z, g))
    fallen = data.qpos[2] < 0.3
    print("segment tracking (mean after 1 s): cmd vx vy wz -> measured")
    for cmd, mean in summary:
        print("  %5.2f %5.2f %5.2f -> %5.2f %5.2f %5.2f" % (*cmd, *mean))
    print("final base_z %.3f%s" % (data.qpos[2], " (FALLEN)" if fallen else ""))
    print("wrote", args.video)


if __name__ == "__main__":
    main()
