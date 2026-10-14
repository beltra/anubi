# A2-W locomotion policy (Isaac Lab / robot_lab)

Trains a wheel-legged velocity policy for the Unitree A2-W with the robot_lab Go2W/B2W
recipe, for `run_a2w_sim.sh --policy <file.npz>`. Sim-only: the real robot is driven by
its onboard sport controller.

Versions: robot_lab v2.3.2, Isaac Lab v2.3.2, Isaac Sim 5.1 (pip), Python 3.11.

## Files

- `unitree_a2w/a2w_asset.py`: articulation and actuators (legs DC motor kp 100 / kd 3 at the
  official A2 torque limits; wheels velocity-damped, estimated limits).
- `unitree_a2w/{rough,flat}_env_cfg.py`, `agents/`: task and PPO config, derived from
  robot_lab's `unitree_b2w` (Apache-2.0). Tasks:
  `RobotLab-Isaac-Velocity-{Flat,Rough}-Unitree-A2W-v0`.
- `export_urdf.sh`: writes `unitree_a2w/description/` (URDF + meshes, not committed) from
  `a2w_description` with articulated legs and continuous wheels.
- `install_into_robot_lab.sh`: symlinks the task into a robot_lab checkout.
- `export_policy_npz.py`: rsl_rl checkpoint -> `.npz` for `a2w_policy_controller`.

## Steps

```bash
# 1. In the ROS workspace (a2w_description built):
training/a2w_robot_lab/export_urdf.sh
# 2. Copy training/a2w_robot_lab to the GPU host, then in the Isaac Lab env:
./install_into_robot_lab.sh ~/robot_lab
cd ~/robot_lab
python scripts/reinforcement_learning/rsl_rl/train.py \
  --task=RobotLab-Isaac-Velocity-Flat-Unitree-A2W-v0 --headless --num_envs 4096
# 3. Export the actor and run it in Gazebo:
python export_policy_npz.py logs/rsl_rl/unitree_a2w_flat/<run>/model_<iter>.pt a2w_flat.npz
scripts/run_a2w_sim.sh --gui --no-lio --policy a2w_flat.npz
```

The Gazebo controller assumes the observation/action layout of this task (57 obs, 16
actions, joint order FR/FL/RR/RL hip-thigh-calf then wheels). Changing observations,
scales or joint order here requires the same change in
`src/simulation/a2w_sim/scripts/a2w_policy_controller.py`.

Install notes for a fresh conda env: pin `setuptools<80` and build `flatdict==4.0.1`
with `--no-build-isolation` before `isaaclab.sh --install`, then restore Isaac Sim 5.1's
pins (`click==8.1.7 psutil==5.9.8 typing_extensions==4.12.2 wrapt==1.16.0`).
