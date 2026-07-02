# g1_locomotion

Self-contained Unitree **G1 AMO** (Adaptive Motion Optimization) whole-body
locomotion driver for anubi, **legs-only** for navigation. It vendors the Psi0
AMO policy assets so anubi has **no runtime dependency on the Psi0 source tree**.

It loads the AMO policy, accepts `{vx, vy, yaw}` velocity commands over a
WebSocket server on **port 8766**, runs policy inference to produce 15-DoF
leg+waist joint targets (**arms held at default**), and talks to the robot over
Unitree DDS (`rt/lowcmd` @ 250 Hz / `rt/lowstate` / `rt/odommodestate`).

> **Scope:** legs + waist (15 DoF, indices 0–14) are policy-driven; the arms
> (indices 15–28, 14 DoF) are held at `default_dof_pos`. There is **no** arm-IK,
> Pinocchio, hands, teleop, camera, or VLA code — all of that was stripped from
> the Psi0 sources during extraction.

---

## Command interface

`ws://127.0.0.1:8766`, JSON messages:

```json
{"vx": 0.3, "vy": 0.0, "yaw": 0.1}
```

| field | meaning              | cap        |
|-------|----------------------|------------|
| `vx`  | forward velocity m/s | `\|vx\| ≤ 0.5` |
| `vy`  | lateral velocity m/s | `\|vy\| ≤ 0.4` |
| `yaw` | yaw rate rad/s       | `\|yaw\| ≤ 0.4` |

(`yaw_rate` is accepted as an alias for `yaw`.) Commands are clamped to the caps
server-side. A dropped/absent connection degrades to the **last-held** command
(it does **not** crash the control loop). The command is held at zero through the
startup blend + stabilize window, then ramped in over `STARTUP_CMD_RAMP_S`.

---

## Bring-up order

1. **`--offline_selftest` first** (no robot, no DDS): validates the observation
   dimension against `amo_jit.pt` and runs one forward pass. **Run this before
   any motor wiring** (see the verification gate below).
   ```bash
   ros2 run g1_locomotion amo_driver --offline_selftest --device cuda
   ```
2. **`--observe_only` next** (DDS subscribe, inference runs, **no motor cmds**):
   confirm obs/action shapes, tilt readings, and command flow look sane on the
   real robot before commanding any motor.
   ```bash
   ros2 run g1_locomotion amo_driver --observe_only --net_if eth0
   ```
3. **Live** (publishes `rt/lowcmd`): only after 1 and 2 pass.
   ```bash
   ros2 run g1_locomotion amo_driver --net_if eth0
   ```

On startup the driver seeds a `JointSmoother` at the **measured** pose so the
first command delta is ≈ 0 (anti-snap), blends from the measured pose to the
policy target over `BLEND_S`, ramps PD gains soft→full over `GAIN_RAMP_S`, and
rate-clamps each joint per tick. Safety guards abort on excessive base tilt
(`> 0.6 rad`) or a stale `rt/lowstate` (`> 0.5 s`).

---

## OBSERVATION VERIFICATION GATE  ⚠️

The ported observation builder (`AmoPolicy.build_observation`, a byte-exact port
of Psi0's `get_ik_observation`) **must** produce a vector whose length equals
`amo_jit.pt`'s expected input dimension. **Two prior analyses disagreed on the
observation length (293 vs 1043).** This package settles it empirically:

```bash
ros2 run g1_locomotion amo_driver --offline_selftest --device cuda
```

This was validated by loading `amo_jit.pt` and running a forward pass directly:

```
obs=1043 ext=2325(25x93) -> action (1, 15)   OK   ✅
obs=293  ...                                  fails ❌
```

**Resolved: obs dim = 1043** (NOT 293). The second policy argument (the
privileged/extra history) is **2325 = 25 × 93** — note it uses
`extra_history_len = 25`, distinct from the proprio history's `history_len = 10`.

### Observation layout (1043)

```
obs_prop (93)   ang_vel*0.25 (3) | rpy[:2] (2) | sin/cos(dyaw) (2) |
                (dof_pos - default) (23) | dof_vel*0.05 (23) |
                last_action (23) | gait sin (2) | adapter_out (15)
obs_demo (17)   demo_dof (8) | vx | vy | 0 | torso_yaw | torso_pitch |
                torso_roll | torso_height ×3
obs_priv (3)    zeros
obs_hist (930)  proprio_history (history_len=10) × 93
                ---------------------------------------------------
total           93 + 17 + 3 + 930 = 1043
```

`extra_hist (2325) = extra_history (extra_history_len=25) × 93` is passed as the
**second** argument to `amo_jit(observation, extra_hist)`.

The 23-element joint subset is `np.r_[0:19, 22:26]` (joints 0–18 then 22–25):
it keeps legs + waist + left arm incl. **left** wrists, and right shoulder×3 +
right elbow, dropping only the **right** wrists. See the module docstring in
[`amo_policy_runtime.py`](g1_locomotion/amo_policy_runtime.py) for the full
element-by-element citation back to the Psi0 source lines.

> **Asset limitation (device):** `amo_jit.pt` was TorchScript-traced with a
> constant baked onto `cuda:0`, so its **forward pass requires a CUDA device**.
> On CPU the dimension gate still validates, but the forward pass raises a
> device-mismatch error; the self-test reports `PARTIAL PASS` and tells you to
> re-run with `--device cuda`. The observation builder itself is device-agnostic.

---

## Key values sourced from Psi0

| value | source |
|-------|--------|
| `default_dof_pos` (29) | `master_whole_body.py:139-145` / `robot_body_ik.py:32-38` |
| `action_scale = 0.25`  | `master_whole_body.py:152` / `robot_body_ik.py:30` |
| `scales_ang_vel = 0.25`, `scales_dof_vel = 0.05` | `master_whole_body.py:129-130` |
| `n_proprio = 93`, `n_priv = 3`, `history_len = 10`, `extra_history_len = 25` | `master_whole_body.py:132-137` |
| raw action clip `±40` | `robot_body_ik.py:454` |
| PD gains `kp` / `kd` (29) | `robot_body.py:201-222` |
| DDS topics `rt/lowcmd` / `rt/lowstate` / `rt/odommodestate` @ 250 Hz | `robot_body.py:27-29, 98` |

`default_dof_pos` (legs L/R hip p/r/y, knee, ankle p/r; waist yaw/roll/pitch;
arms shoulder p/r/y, elbow, wrist r/p/y):

```
[-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   # left leg
 -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   # right leg
  0.0, 0.0, 0.0,                   # waist
  0.5, 0.0, 0.2, 0.3, 0.0, 0.0, 0.0,   # left arm  (held)
  0.5, 0.0, -0.2, 0.3, 0.0, 0.0, 0.0]  # right arm (held)
```

---

## Python dependencies (pip)

These are **not** rosdep/apt packages — install into the deployment env:

```bash
pip install torch numpy scipy unitree_sdk2py websockets
```

| dep | needed for |
|-----|------------|
| `torch` | loads `amo_jit.pt` / `adapter_jit.pt` TorchScript; runs inference (**CUDA** for the forward pass — see asset limitation) |
| `numpy` | observation math (also a ROS `python3-numpy` dep) |
| `scipy` | only if you extend the driver with `scipy.spatial.transform`; the core path uses a hand-rolled quat→euler |
| `unitree_sdk2py` | Unitree DDS I/O (`rt/lowcmd`/`rt/lowstate`) — **only on the robot**; imported lazily so offline modes work without it |
| `websockets` | the `:8766` command server (optional; absent ⇒ command stays zero) |

---

## Files

| file | role |
|------|------|
| `g1_locomotion/amo_policy_runtime.py` | `AmoPolicy`: loads assets, byte-exact `get_ik_observation` port, adapter + policy inference (`is_teleop=False`), 15-DoF scaled action onto `default_dof_pos` |
| `g1_locomotion/robot_dds_io.py` | `G1DDSInterface`: Unitree DDS transport (sub `rt/lowstate`+`rt/odommodestate`, pub `rt/lowcmd` 29-DoF PD @ 250 Hz); `unitree_sdk2py` imported lazily |
| `g1_locomotion/amo_driver.py` | main driver: CommandSource + WS `:8766`, JointSmoother, Pacer @ 50 Hz inference, safety guards, `--observe_only`, `--offline_selftest` |
| `g1_locomotion/assets/*.pt` | vendored AMO assets copied verbatim from Psi0 (`amo_jit.pt`, `adapter_jit.pt`, `adapter_norm_stats.pt`) |

---

## Unresolved verification gates (validate on hardware)

- **CPU inference unsupported.** `amo_jit.pt` carries a `cuda:0` constant; CPU
  forward pass fails. Re-export the JIT if CPU is ever required.
- **50 Hz inference vs 250 Hz publish.** Inference ticks at 50 Hz (Psi0
  `dt=0.02` for `target_yaw`/`gait_cycle` integration) while DDS streams the
  latest target at 250 Hz. Confirm this cadence matches Psi0's effective control
  rate on the robot (TODO(verify) in `amo_driver.py`).
- **No damping on shutdown.** `G1DDSInterface.shutdown()` stops the publisher
  under the last PD target; it does **not** command a damping pose. The hardware
  damp combo remains the real e-stop (TODO(verify) in `robot_dds_io.py`).
- **Torso reference held constant.** Navigation holds `torso_height=0.75`, level
  (matching Psi0 `maintain_standing`). Walking gaits were trained with this
  reference; confirm on hardware.
- **Quaternion source / convention.** `read_state()` uses the LowState IMU
  quaternion (wxyz) as Psi0 does; verify the IMU frame on the target unit.
```
