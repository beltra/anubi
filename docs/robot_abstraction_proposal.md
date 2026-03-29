# Robot Abstraction Proposal

Goal: make Go2, D1 Ultra, and future robots share ~80-90% of the navigation stack while isolating hardware differences.

## 1) Target Architecture

Proposed package boundaries:

- `robot_description_<robot>`  
  URDF/Xacro, static frame tree, sensor extrinsics.

- `robot_adapter_<robot>`  
  Hardware/SDK bridge only. Converts native state/commands to standard ROS contract.

- `robot_bringup_<robot>`  
  Robot-specific launch entrypoint that composes adapter + shared core launch.

- `sensor_pipeline_core`  
  Shared preprocessing (cloud filters, pointcloud-to-scan, optional IMU conditioning).

- `localization_core`  
  `slam_toolbox` and `robot_localization` launch/config templates.

- `navigation_core`  
  Shared Nav2 launch + base parameter templates.

- `robot_overlays/<robot>/`  
  Per-robot YAML overlays (footprint, velocity limits, topic remaps, covariance, frame aliases).

Current repo mapping toward this:
- Existing `robot_nav` ~= `navigation_core` + part of `localization_core`.
- Existing `go2_bringup` ~= `robot_adapter_go2` + `robot_bringup_go2`.
- Existing `sensor_models` ~= partial sensor abstraction.

---

## 2) Contract Ownership

## 2.1 Topic contract (external)

- `/odom_raw` (`nav_msgs/Odometry`) from robot adapter
- `/imu/data` (`sensor_msgs/Imu`) from robot adapter or IMU adapter
- `/lidar_odom` (`nav_msgs/Odometry`, optional) from lidar odom module
- `/odom` (fused) from EKF
- `/scan` or `/points` from sensor pipeline
- `/cmd_vel` from Nav2 to robot adapter

## 2.2 TF ownership

- `map->odom`: `slam_toolbox` (or AMCL in localization mode)
- `odom->base_footprint` (or `base_link`): EKF when fusion enabled
- `base_footprint->base_link` and sensor links: `robot_state_publisher`
- Raw adapter should **not** publish `odom->base_*` TF when EKF owns it

---

## 3) How Go2 and D1 Plug In

## 3.1 Go2

- Keep `go2_hw_bridge` for SDK integration.
- Emit `/odom_raw` and optionally disable TF via `publish_tf=false` when EKF enabled.
- Add/standardize IMU output as `/imu/data` with covariance.
- Reuse shared `localization_core` + `navigation_core`.

## 3.2 D1 Ultra

- Implement `d1_hw_bridge` (same external contract as Go2 adapter).
- Provide `d1_description` with same canonical frame names.
- Add D1 overlay YAMLs:
  - footprint and speed limits
  - odom and IMU covariance
  - lidar filter/scan slicing params
- Reuse shared core launches unchanged.

---

## 4) Recommended Topic/Frame Conventions

## 4.1 Frames

- Keep canonical names:
  - `map`, `odom`, `base_footprint`, `base_link`, `lidar_link`, `imu_link`
- If robot-native frames differ, resolve at adapter layer (remap/transform), not in Nav2 core.

## 4.2 Topics

- Prefer relative topics in node code, namespace via launch.
- Avoid hardcoded leading `/` in reusable core packages.
- Keep stable externally-visible contract regardless of vendor SDK internals.

---

## 5) Staged Migration Plan

## Stage A (low risk, immediate)

1. Keep current bringups unchanged.
2. Use new bridge knobs (`odom_topic`, `publish_tf`) and optional EKF launch.
3. Add D1 TODO issue list and ownership.

## Stage B (fusion-ready baseline)

1. Switch adapters to publish `/odom_raw`.
2. Add calibrated covariance to adapter odom and IMU streams.
3. Launch EKF in robot bringup (initially with body odom + IMU, lidar odom optional).
4. Remap EKF output to `/odom`.

## Stage C (robot overlays)

1. Split `nav2_params` into:
  - common template
  - `go2` overlay
  - `d1` overlay
2. Split scan and EKF params similarly.

## Stage D (multi-robot robustness)

1. Remove remaining absolute topic hardcoding in shared packages.
2. Add namespace-first launch patterns.
3. Add launch tests for duplicate TF and contract compliance.

---

## 6) Minimal Repository Reorg (smallest viable)

Without big package renames, this repo can evolve in-place:

- Keep `robot_nav`, add subfolders:
  - `config/common/`
  - `config/go2/`
  - `config/d1/`
  - `launch/localization/`
  - `launch/navigation/`
- Keep `go2_bringup`, add future `d1_bringup`.
- Add `docs/contracts/robot_interface.md` as machine-readable checklist.

This achieves abstraction benefits with minimal disruption to existing users.

