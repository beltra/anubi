# LiDAR Processing

This context defines the language for raw and corrected LiDAR perception in Anubi.
It exists because downstream navigation treats the corrected cloud as the canonical perception input.

## Language

**Raw LiDAR Cloud**:
The pointcloud published in the physical LiDAR frame before Anubi correction.
_Avoid_: Sensor cloud, original cloud

**Corrected LiDAR Cloud**:
The single downstream pointcloud after self filtering, gravity alignment, and optional deskew.
_Avoid_: Filtered cloud, undistorted-only cloud

**Gravity-Aligned LiDAR Frame**:
A LiDAR-centered frame that tracks sensor position while holding roll and pitch level with gravity.
_Avoid_: Stabilized frame, level frame

**LiDAR Mount Rotation**:
The physical rotation from `base_link` to `lidar_link`.
_Avoid_: Sensor rotation

**Missing-State Drop**:
The policy of withholding corrected clouds when required IMU or odometry state is stale.
_Avoid_: Passthrough fallback

## Relationships

- A **Raw LiDAR Cloud** produces zero or one **Corrected LiDAR Cloud**.
- A **Corrected LiDAR Cloud** is expressed in the **Gravity-Aligned LiDAR Frame** when IMU state is available.
- The **LiDAR Mount Rotation** must be inverted when mapping points from the LiDAR frame into base or gravity-aligned frames.
- **Missing-State Drop** prevents raw clouds from masquerading as corrected perception.

## Example Dialogue

> **Dev:** "Can we publish raw data on the corrected topic until IMU arrives?"
> **Domain expert:** "No. The **Corrected LiDAR Cloud** has a stronger contract; use **Missing-State Drop** so Nav2 does not see inconsistent obstacle geometry."

## Flagged Ambiguities

- "undistorted cloud" can imply deskew only; resolved: **Corrected LiDAR Cloud** includes self filtering, trunk stabilization, and optional hardware deskew.
