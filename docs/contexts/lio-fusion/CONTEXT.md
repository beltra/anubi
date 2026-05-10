# LIO Fusion

This context defines optional lidar-inertial odometry language.
It exists so LIO can improve odometry without taking over the core TF ownership contract.

## Language

**Body Odometry**:
Odometry derived from the robot body state or simulated locomotion model.
_Avoid_: Wheel odom

**LiDAR Odometry**:
Odometry estimated by lidar-inertial odometry or scan matching.
_Avoid_: LIO pose

**Reliable LiDAR Odometry**:
LiDAR odometry with explicit covariance suitable for EKF consumption.
_Avoid_: Raw DLIO odom

**Fused Odometry**:
The EKF output consumed by SLAM and Nav2.
_Avoid_: Filtered odom when the source is unclear

**TF Owner**:
The one component responsible for publishing a transform in the live TF tree.
_Avoid_: TF source

## Relationships

- **Body Odometry** and **Reliable LiDAR Odometry** may feed **Fused Odometry**.
- **LiDAR Odometry** must be converted to **Reliable LiDAR Odometry** before EKF fusion.
- The EKF is the **TF Owner** for `odom -> base_footprint`.
- DLIO is optional and must not become the direct **TF Owner** for the navigation odom transform.

## Example Dialogue

> **Dev:** "DLIO looks better in sim; should Nav2 consume `/lidar_odom` directly?"
> **Domain expert:** "No. Promote it to **Reliable LiDAR Odometry** first, then let the EKF publish **Fused Odometry**."

## Flagged Ambiguities

- "odom" can mean **Body Odometry**, **LiDAR Odometry**, or **Fused Odometry**; use the precise source when discussing fusion.
