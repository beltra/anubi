# Fusion Readiness Checklist

Legend: `PASS` = ready now, `PARTIAL` = possible with bounded changes, `FAIL` = missing critical prerequisite.

| Area | Status | Notes |
|---|---|---|
| Raw odom topic separation (`/odom_raw`) | PARTIAL | Bridge now supports `odom_topic`, but default runtime still uses `/odom` |
| Fused odom topic path (`/odometry/filtered` or `/odom`) | PARTIAL | Optional EKF launch exists; not integrated in default bringups |
| Covariance quality (raw odom) | FAIL | `go2_hw_bridge` does not populate odom covariance |
| Covariance quality (IMU) | FAIL | No enforced `/imu/data` adapter/covariance policy in real bringup |
| TF ownership clarity (`map->odom`, `odom->base`) | PARTIAL | Concept clear; fusion ownership not active by default |
| Duplicate odom TF prevention | PARTIAL | Bridge now has `publish_tf` switch; must be set correctly in fusion mode |
| Lidar odometry insertion point | PARTIAL | `/lidar_odom` path present in EKF template; no lidar odom node implementation yet |
| Nav2 consuming fused odom safely | PARTIAL | Safe after topic switch + single TF publisher rule |
| SLAM compatibility with fused odom | PASS | `slam_toolbox` already expects odom/base TF chain |
| Launch composability for estimator | PARTIAL | `robot_nav/launch/ekf.launch.py` added, not wired by default |
| Topic hygiene (`/odom`, `/scan`, `/unilidar/cloud`, `/cmd_vel`) | PARTIAL | Works today, but many absolute topic names reduce flexibility |
| Frame hygiene (`map`, `odom`, `base_footprint`, etc.) | PARTIAL | Canonical frames exist; parameterization is limited |
| Multi-robot namespace readiness | FAIL | Absolute topics and partial namespace handling risk collisions |
| Go2 readiness for fusion | PARTIAL | Close after covariance + IMU normalization + default bringup integration |
| D1 Ultra readiness | FAIL | No D1 hardware adapter; `d1_sim` still stub |
| Generic future robot extensibility | PARTIAL | Good package split intent, but contracts not fully operationalized |
| Rosbag-based debugability (raw vs lidar vs fused) | PARTIAL | Standard topics available; no dedicated validation scripts |

## Must-do Before Production Fusion

1. Publish realistic covariance on body odom and IMU.
2. Standardize raw topics (`/odom_raw`, `/imu/data`, `/lidar_odom`).
3. Ensure EKF is the only `odom->base_*` TF broadcaster in fusion mode.
4. Add D1 adapter + overlay configs before claiming dual-robot support.

