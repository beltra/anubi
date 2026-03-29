# Odom Fusion Readiness Audit

Date: 2026-03-29  
Repository: `/ws/anubi`  
Scope: Readiness for lidar + body odometry fusion for Go2 and D1 Ultra, and extensibility to future robots.

## 1) Executive Verdict

- Lidar + body odometry fusion readiness today: **PARTIAL (not production-ready yet)**
- Ready for both Go2 and D1 Ultra today: **NO**
- Architecturally extensible to future robots today: **PARTIAL**

Why:
- The stack has a clear Nav2/SLAM split and consistent baseline frame names (`map`, `odom`, `base_footprint`, `base_link`, `lidar_link`, `imu_link`) in design intent.
- But runtime odometry is still treated as a single authoritative `/odom` in current bringups; no end-to-end raw/fused separation is active by default.
- D1 Ultra hardware adapter does not exist, and `d1_sim` is explicitly a stub with placeholders.

---

## 2) Repository Architecture Map

## 2.1 Package map and ownership

| Package | Role | Classification | Evidence |
|---|---|---|---|
| `go2_bringup` | Real hardware bridge + master bringup | **Go2-specific** | `src/go2_bringup/src/go2_hw_bridge.cpp`, `src/go2_bringup/launch/go2_bringup.launch.py` |
| `go2_description` | Real robot URDF frames | **Go2-specific** | `src/go2_description/urdf/go2.urdf.xacro` |
| `go2_sim` | Gazebo Go2 model + planar odom plugin | **Go2-specific** | `src/go2_sim/urdf/go2_sim.urdf.xacro` |
| `d1_sim` | Gazebo D1 model | **D1 placeholder (not complete)** | `src/d1_sim/urdf/d1_sim.urdf.xacro` TODO stub |
| `robot_nav` | Shared Nav2 + SLAM launch/config | **Mostly robot-agnostic, but Go2-biased params** | `src/robot_nav/config/nav2_params.yaml` (Go2 footprint/tuning comments) |
| `robot_sim` | Shared sim bringup wrapper for go2/d1 | **Partially generic** | `src/robot_sim/launch/sim_bringup.launch.py` supports `robot:=go2|d1` but shared defaults are Go2-biased |
| `sensor_models` | Reusable Gazebo sensor macro(s) | **Generic framework, currently L1-specific** | `src/sensor_models/urdf/l1_lidar.urdf.xacro` |
| `robot_common_interfaces` | Shared message + declared adapter contract | **Generic contract layer** | `src/robot_common_interfaces/README.md` |
| `robot_safety` | Velocity gate and placeholders | **Mostly generic** | `src/robot_safety/scripts/velocity_limiter_node.py` |
| `unitree_go`, `unitree_api` | Unitree message/API definitions | **Go2/vendor-specific** | `src/unitree_go/msg/*`, `src/unitree_api/msg/*` |
| `sim_worlds`, `sim_scenarios` | Simulation environment support | **Generic sim support** | `src/sim_worlds/*`, `src/sim_scenarios/*` |

## 2.2 Bringup graph (real Go2)

`go2_bringup.launch.py` starts:
1. `robot_state_publisher`
2. `go2_hw_bridge`
3. optional safety
4. `slam.launch.py` (PointCloud2→LaserScan + slam_toolbox)
5. `nav2.launch.py`
6. RViz  

Evidence: `src/go2_bringup/launch/go2_bringup.launch.py:5-11, 99-166`

## 2.3 Bringup graph (sim)

`sim_bringup.launch.py` starts Gazebo + RSP + spawn + SLAM + Nav2; odom comes from Gazebo planar plugin, not hardware bridge.  
Evidence: `src/robot_sim/launch/sim_bringup.launch.py:17-19, 23-27, 287-311`

---

## 3) Odometry Pipeline Audit

## 3.1 Odometry publishers found

1. `go2_hw_bridge` publishes odom message and odom TF:
- Odom topic publisher now parameterized via `odom_topic` (default `odom`)
- TF publication now parameterized via `publish_tf` (default `true`)
- Frame IDs: `odom_frame` / `base_frame`
- No covariance assignment in code (defaults remain zero-initialized)

Evidence:
- `src/go2_bringup/src/go2_hw_bridge.cpp:72-76, 85-88, 96, 163-165, 183, 185-198`
- `src/go2_bringup/config/go2_params.yaml:18-27`

2. Gazebo planar plugin publishes `/odom` and `odom->base_footprint` TF in sim:
- `publish_odom=true`, `publish_odom_tf=true`
- frame: `odometry_frame=odom`, `robot_base_frame=base_footprint`

Evidence:
- `src/go2_sim/urdf/go2_sim.urdf.xacro:252-265`
- `src/d1_sim/urdf/d1_sim.urdf.xacro:95-108` (stub but same pattern)

## 3.2 Classification of motion sources

| Source | Classification |
|---|---|
| `go2_hw_bridge` odom from `SportModeState` | **Raw body odometry** (currently used as Nav2 odom) |
| Gazebo planar odom | **Raw simulated body odometry** |
| `slam_toolbox` | **Map-relative localization (`map->odom`)** |
| Nav2 controller (`/cmd_vel`) | **Motion command source** |
| EKF/UKF fusion node | **Not integrated in default bringups** |

## 3.3 Competing odom sources / hidden assumptions

- Real bringup: single odom source (`go2_hw_bridge`) expected.
- Sim bringup: single odom source (planar plugin) expected.
- The stack historically assumes `/odom` is already final navigation odometry.
- No active `/odom_raw` vs fused `/odom` split in default launch paths.

Evidence:
- `src/go2_bringup/launch/go2_bringup.launch.py` has no EKF include.
- `src/robot_sim/launch/sim_bringup.launch.py` relies on planar plugin odom.
- Nav2 consumes `/odom` directly: `src/robot_nav/config/nav2_params.yaml:377,500`

## 3.4 Covariance readiness

- `go2_hw_bridge` does not set pose/twist covariance fields.
- `SportModeState` message has no covariance fields.
- This is a blocker for robust robot_localization tuning; default zero covariance leads to over-trusting that source.
- Sim planar plugin has only coarse covariance params (`covariance_x/y/yaw`), not full per-axis calibrated covariance.

Evidence:
- `src/go2_bringup/src/go2_hw_bridge.cpp:158-183` (no covariance assignment)
- `src/unitree_go/msg/SportModeState.msg:1-14`
- `src/go2_sim/urdf/go2_sim.urdf.xacro:262-264`

---

## 4) Lidar Pipeline Audit

## 4.1 Current path

`/unilidar/cloud` (`PointCloud2`) -> `pointcloud_to_laserscan` -> `/scan` -> `slam_toolbox`  
Also `/unilidar/cloud` feeds Nav2 voxel layers directly.

Evidence:
- `src/robot_nav/launch/slam.launch.py:96-109`
- `src/robot_nav/config/scan_from_cloud.yaml:17-42`
- `src/robot_nav/config/nav2_params.yaml:246-269, 334-355`

## 4.2 Suitability for scan-matching / lidar odometry

- Usable as baseline input, but currently no dedicated lidar odom output node in repo (`/lidar_odom` absent before added EKF placeholder config).
- Real stack lacks cloud self-filtering used in sim (`cloud_self_filter` only in `robot_sim`, and only for Go2 path).
- Height slicing is fixed (`min_height=-0.06`, `max_height=0.06`) and assumes correct `lidar_link` orientation; any tilt mismatch will degrade scan quality.

Evidence:
- No ICP/NDT/scan-matcher node package in repo.
- `src/robot_sim/launch/sim_bringup.launch.py:274-285` (`cloud_self_filter` only under `is_go2`)
- `src/robot_nav/config/scan_from_cloud.yaml:21, 28-29`

## 4.3 Generic vs Go2 coupling

- Topic names are hardcoded globally (`/unilidar/cloud`, `/scan`) in SLAM launch and Nav2 params.
- This can be adapted via per-robot params/launch remaps, but not centrally abstracted yet.

Evidence:
- `src/robot_nav/launch/slam.launch.py:106-107`
- `src/robot_nav/config/nav2_params.yaml:248,267,336,354`

---

## 5) TF Tree Audit

## 5.1 Intended TF ownership (mostly correct)

- `map->odom`: `slam_toolbox`
- `odom->base_footprint`: raw odom adapter (Go2 bridge or planar plugin)
- fixed sensor chain: URDF through `robot_state_publisher`

Evidence:
- `src/robot_common_interfaces/README.md:16-26`
- `src/robot_nav/config/slam_toolbox_params.yaml:30-33`

## 5.2 Issues and risks

1. Raw vs fused TF ownership not yet active by default:
- Existing bringups do not start EKF, so raw odom remains owner.
- Needed for fusion: raw source publishes `/odom_raw` and usually **does not** publish odom TF when EKF owns TF.

2. Hardcoded frame assumptions:
- Many configs fix `base_footprint`, `odom`, `map`.
- Good for consistency, but not parameterized at a single abstraction layer.

3. AMCL TF risk is low in current launch:
- AMCL block exists in params with `tf_broadcast=true`, but `nav2.launch.py` delegates to `navigation_launch.py` that does not start AMCL/map_server.

Evidence:
- AMCL params: `src/robot_nav/config/nav2_params.yaml:31-70`
- nav2_bringup launch behavior: `/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py:43-49,111-193`

---

## 6) Nav2 Coupling Audit

## 6.1 Current Nav2 assumptions

- Odom topic consumed as `/odom`
- Base frame is `base_footprint`
- Global frame `map`, local costmap frame `odom`
- Costmaps consume `/scan` and `/unilidar/cloud` (or filtered sim cloud variant)

Evidence:
- `src/robot_nav/config/nav2_params.yaml:211-213, 292-294, 377, 500`

## 6.2 Go2-specific coupling inside "shared" config

- Footprint and velocity limits are Go2 values.
- Comments explicitly mention Go2 dimensions and tuning assumptions.

Evidence:
- `src/robot_nav/config/nav2_params.yaml:154-160, 221-229, 303-311`
- Similar in sim params: `src/robot_sim/config/nav2_sim_params.yaml:223-230`

## 6.3 Safety of switching Nav2 to fused odom

- Safe if done with proper TF ownership and topic swap:
  - Raw body odom -> `/odom_raw`
  - EKF output -> `/odom` (or remapped)
  - Single odom TF broadcaster
- Not safe if raw bridge and EKF both publish `odom->base_*`.

---

## 7) Robot Abstraction Audit

## 7.1 What exists

- Good concept-level adapter contract (`robot_common_interfaces/README.md`)
- Clear package split between hardware (`go2_bringup`) and nav (`robot_nav`)
- Sim launcher supports robot selector (`go2|d1`)

## 7.2 What is missing for true abstraction

- No D1 hardware adapter package.
- D1 simulation model is marked stub.
- No per-robot overlay hierarchy for nav/localization config.
- Topic/frame conventions are not centralized as a machine-readable config contract.
- Multi-robot namespace hygiene is weak (many absolute topic paths).

Evidence:
- D1 stub: `src/d1_sim/urdf/d1_sim.urdf.xacro:4-20`
- Absolute topic defaults: `src/go2_bringup/config/go2_params.yaml`, `src/robot_nav/config/nav2_params.yaml`

---

## 8) D1 Ultra Readiness Analysis

Assumed D1 provides body odom, imu, scan/cloud, TF, motion command bridge.

## 8.1 Must change

1. Implement D1 hardware adapter package (equivalent to `go2_hw_bridge` contract).
2. Finish `d1_sim` model (currently stub placeholders).
3. Provide D1-specific nav and localization overlay params (footprint, limits, sensor rates, covariances).
4. Establish raw/fused odom separation in D1 bringup (`/odom_raw` and EKF output policy).

## 8.2 Should change

1. Add D1-specific cloud/scan preprocessing if sensor geometry differs.
2. Add D1 IMU normalization adapter to publish `sensor_msgs/Imu` on `/imu/data` with realistic covariance.
3. Avoid absolute topics to enable clean namespace deployments.

## 8.3 Nice to have

1. Robot capability manifest (YAML) consumed by generic bringup.
2. Automated launch tests verifying TF ownership and topic contract.
3. Rosbag validation scripts for body-vs-lidar-vs-fused trajectory comparison.

---

## 9) Fusion Readiness Checklist (expanded)

| Item | Status | Notes |
|---|---|---|
| Separate raw odom topic namespace available | **PARTIAL** | Bridge now parameterized (`odom_topic`), default still `odom` |
| Separate fused odom namespace available | **PARTIAL** | EKF placeholder added (`ekf.launch.py`), not integrated in default bringups |
| Consistent covariance strategy | **FAIL** | Body odom covariance missing; IMU pipeline not standardized in real bringup |
| TF ownership clearly defined | **PARTIAL** | Concept is clear; default runtime still raw odom owner, no active fusion ownership |
| `map->odom` ownership clear | **PASS** | slam_toolbox currently owns map->odom in active stack |
| Launch sequencing for estimator possible | **PARTIAL** | New optional EKF launch exists, but not wired into master bringups |
| IMU topic availability abstracted | **FAIL** | No dedicated IMU adapter contract implementation in hardware bringup |
| Lidar odom insertion point exists | **PARTIAL** | Added placeholder input `/lidar_odom` in EKF params; no lidar odom node implementation |
| Nav2 can consume fused `/odom` safely | **PARTIAL** | Yes if remapped and duplicate TF disabled |
| Bagging/debug topics for validation | **PARTIAL** | Standard topics exist, but no dedicated validation tooling or explicit raw/fused compare convention |

---

## 10) High-Level Questions Answered

1. Clean separation raw/fused/localization/navigation consumers?  
**No (partial)**. Localization and navigation are separated, but raw vs fused odom is not active by default.

2. Topics/frames/launch layout abstract enough for multi-robot without copying?  
**Partial**. Package split helps, but absolute topic names and Go2-tuned shared params limit plug-and-play reuse.

3. Can lidar-derived odom be added without breaking Nav2/SLAM/TF?  
**Yes, with controlled changes**: keep slam_toolbox on map->odom, set raw odom to `/odom_raw`, and ensure single odom TF owner.

4. Is current Go2 integration too hardware-specific to be reusable?  
**Bridge logic is reusable pattern-wise, implementation is Go2-specific**.

5. What is needed for clean D1 support?  
Not just description: requires a D1 adapter, D1 config overlays, and finishing `d1_sim`.

6. Minimal architectural changes for generic future robots?  
Introduce explicit adapter/localization core boundaries and per-robot overlays (detailed in `robot_abstraction_proposal.md`).

7. Should robot_localization be next step now?  
**Yes, but gated** by covariance cleanup + raw/fused topic/TF ownership policy. Do not wire EKF as default until those are done.

---

## 11) Blockers

1. No calibrated covariance on body odom source.
2. No guaranteed standardized IMU stream in real hardware bringup (`/imu/data` contract not enforced).
3. No D1 hardware adapter package.
4. `d1_sim` still placeholder/stub.
5. Shared nav params are still Go2 footprint/tuning defaults.
6. No integrated lidar odometry producer (`/lidar_odom`) in repo.

---

## 12) Minimal Changes Applied During Audit

These were added as low-risk readiness scaffolding without altering default stack behavior:

1. `go2_hw_bridge` now supports:
- configurable `odom_topic` (default `odom`)
- configurable `publish_tf` (default `true`)

Files:
- `src/go2_bringup/src/go2_hw_bridge.cpp`
- `src/go2_bringup/config/go2_params.yaml`

2. Added optional EKF scaffold:
- `src/robot_nav/launch/ekf.launch.py`
- `src/robot_nav/config/ekf_fusion_params.yaml`
- `src/robot_nav/package.xml` now depends on `robot_localization`

This enables a staged migration path to:
- raw adapter -> `/odom_raw`
- EKF -> `/odom` (via remap)
- single odom TF publisher.

---

## 13) Recommended Migration Plan (minimal viable)

Phase 1 (contract hardening):
1. Standardize raw topics: `/odom_raw`, `/imu/data`, `/lidar_odom`.
2. Add realistic covariance in body odom adapter(s).
3. Define TF ownership policy: raw adapters may disable odom TF when EKF active.

Phase 2 (Go2 fusion):
1. Launch EKF from `robot_nav/launch/ekf.launch.py`.
2. Feed `/odom_raw` (+ IMU, optional lidar odom).
3. Remap EKF output to `/odom`; keep Nav2 config unchanged.

Phase 3 (D1 enablement):
1. Implement `d1_bringup` adapter with same external contract.
2. Complete `d1_sim` URDF.
3. Add D1 overlay params for Nav2 + EKF.

Phase 4 (generalization):
1. Introduce per-robot overlay tree and capability manifest.
2. Add launch tests for TF ownership and topic contracts.
3. Add rosbag replay validation pipelines.

---

## 14) Risk Analysis

| Risk | Impact | Likelihood | Mitigation |
|---|---|---|---|
| Duplicate `odom->base_*` TF when EKF is enabled | Nav2 instability, TF jumps | High | Disable raw TF (`publish_tf=false`) when EKF publishes TF |
| Zero/invalid covariance from raw odom/IMU | EKF overconfidence/divergence | High | Populate realistic covariance before production fusion |
| D1 integration via copy-paste instead of adapter contract | Long-term maintenance debt | Medium | Enforce adapter interface + overlay config policy |
| Hardcoded absolute topics in multi-robot runs | Topic collisions | Medium | Move to relative topics + namespace-aware launch remaps |
| Lidar frame/tilt mismatch | Poor scan matching and map quality | Medium | Validate `lidar_link` extrinsics and scan slicing per robot |

