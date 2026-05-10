# Real LiDAR orientation and IMU processing

The real Go2 LiDAR orientation is derived from live Unitree topics, not from the nominal mount description. Matching `/utlidar/cloud` points to `/utlidar/cloud_base` by `ring` and `time` gives:

```text
R raw_utlidar_lidar_to_base =
[[-0.9659793436, 0.0,  0.2586192332],
 [ 0.0,          1.0,  0.0],
 [-0.2586192332, 0.0, -0.9659793436]]
```

The residual of this point match was below 1e-6 m, so the runtime LiDAR correction uses this rotation. With `mount_rotation_mode: inverse`, `distortion_corrector` stores the inverse rotation as `sensor_mount_rpy: [0.0, -2.88000012056847, 0.0]`.

The reference stack in `/ws/autonomy_stack_go2` also rotates the raw cloud by approximately `Ry(pi - 15.1 deg)`, then applies the `0.046825 m` vertical camera offset and a body-return filter. Its LiDAR IMU path is not just a static frame transform: it flips raw Y/Z axes, applies a 15.1 degree Y rotation, subtracts `imu_calib_data.yaml` biases, compensates Z-rate cross-axis coupling, and publishes a gyro-only IMU for Point-LIO.

Anubi's default Nav2/SLAM pipeline does not need that LiDAR IMU preprocessor because LiDAR correction uses the body IMU on `/imu/data` for gravity alignment and `use_lio` is off by default. If `/lidar/imu` becomes the active LIO input, add a real LiDAR IMU preprocessing node rather than replacing the old calibration with a static TF.

The Unitree L1 point pattern is non-repetitive, so one `/utlidar/cloud` frame is too sparse for a 2D LaserScan slice. A live probe on the real robot showed `/scan` averaging about 40 finite bins out of 720 with gaps over 300 bins. Real bringup therefore runs `scan_cloud_accumulator` before `pointcloud_to_laserscan`; it publishes `/lidar/points_scan_accumulated` from a short rolling window of corrected clouds. The latest corrected cloud remains the source for STVL and collision monitoring.
