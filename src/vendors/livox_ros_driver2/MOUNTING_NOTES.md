# Livox Mid-360 Mounting & Integration Notes (Unitree G1 / anubi feat/g1)

> This file is anubi-specific integration documentation. It does NOT modify any
> upstream driver source. All upstream `LICENSE.txt` and file headers are
> preserved verbatim. This package is for **HARDWARE only**; simulation uses a
> Gazebo GPU-lidar instead and does **not** require this driver.

## 1. Inverted mount and the `roll: 180` extrinsic

On the Unitree G1 the Livox Mid-360 is mounted **upside-down (inverted)**.
To compensate, the driver's per-lidar extrinsic in `config/MID360_config.json`
applies a 180-degree roll so the published point cloud arrives **upright** in
the `mid360_link` frame:

```
"extrinsic_parameter" : {
  "roll": 180.0,   <-- uprights the inverted cloud
  "pitch": 0.0,
  "yaw": 0.0,
  "x": 0, "y": 0, "z": 0
}
```

Do **not** change these values — the 180-degree roll is required because of the
physical mounting orientation. The extrinsic is applied by the Livox driver to
the **point cloud only**.

## 2. IMU is published RAW (still inverted) — DLIO corrects it

IMPORTANT: The Livox driver applies the extrinsic to the point cloud, but the
IMU on `/livox/imu` is published **RAW**, i.e. it is **still in the inverted
(upside-down) sensor frame**. The downstream DLIO odometry node is responsible
for rotating the IMU into the upright body frame using its baselink-to-imu
extrinsic:

```
baselink2imu = R_x(180)   # rotation about X by 180 degrees
```

So: cloud is uprighted by the driver (`roll:180`), IMU is uprighted by DLIO
(`baselink2imu = R_x(180)`). Both compensate for the same inverted mount, but at
different stages of the pipeline. Keep them consistent — if the physical mount
ever changes, both must change together.

## 3. Network configuration (must match the robot network)

The IPs in `config/MID360_config.json` must match the Unitree G1 robot network.
Unitree defaults:

| Role            | IP                | Notes                                  |
|-----------------|-------------------|----------------------------------------|
| Lidar (Mid-360) | `192.168.123.120` | `lidar_configs[0].ip`                  |
| Host (compute)  | `192.168.123.222` | `host_net_info` cmd/push/point/imu IPs |

If the driver reports no data, first confirm these IPs match the actual robot
network and that the host NIC is on the `192.168.123.0/24` subnet.

## 4. Livox-SDK2 is an EXTERNAL system dependency (hardware only)

This driver links against **Livox-SDK2**, which is an **external system library
that is NOT vendored here and must be installed separately on the real robot.**
It is **NOT needed for simulation.**

- `CMakeLists.txt` locates it via `LIVOX_SDK2_ROOT` (env var or the default
  `/ros_ws/Livox-SDK2`), searching `${LIVOX_SDK2_ROOT}/build/sdk_core`,
  `${LIVOX_SDK2_ROOT}/lib`, and `/usr/local/lib` for
  `livox_lidar_sdk_shared`, plus `${LIVOX_SDK2_ROOT}/include` (or
  `/usr/local/include`) for `livox_lidar_api.h` / `livox_lidar_def.h`.
- Both the library and the headers are `REQUIRED` — the package will **NOT
  build without Livox-SDK2 present.** This is expected on hosts that only run
  simulation; do not attempt to build this package there.
- Install on the robot from https://github.com/Livox-SDK/Livox-SDK2 (build +
  `make install`), or set `-DLIVOX_SDK2_ROOT=/path/to/Livox-SDK2`.

## 5. Build, topics, and DLIO point-cloud format

### Build
Built via the standard ROS 2 workflow. Upstream provides `build.sh`, invoked as:

```
./build.sh humble      # (or: ./build.sh ROS2)
```

`build.sh` swaps in `package_ROS2.xml` -> `package.xml`, copies
`launch_ROS2/` -> `launch/`, then runs
`colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble`.

### Executable produced
- `livox_ros_driver2_node` — the driver node, registered as the composable
  component `livox_ros::DriverNode` (via `rclcpp_components_register_node`).
  Launch with `launch_ROS2/msg_MID360_launch.py`.

### Published topics (with `multi_topic = 0`)
- PointCloud2 / CustomMsg : **`/livox/lidar`**
- IMU (`sensor_msgs/Imu`) : **`/livox/imu`**

### Point-cloud format for DLIO per-point timestamp auto-detection
DLIO needs **per-point timestamps** to deskew the scan. To get the native Livox
point format (which carries per-point time/offset), set in the launch file
(`msg_MID360_launch.py`):

```
xfer_format = 1   # 1 = customized Livox pointcloud format (livox_ros_driver2/CustomMsg)
                  # 0 = standard PointCloud2 (PointXYZRTL)
```

`xfer_format = 1` publishes the Livox `CustomMsg` format, which DLIO
auto-detects as the Livox sensor type and uses for per-point timestamp
deskewing. (The bundled `MID360_config.json` also sets `pcl_data_type: 1`.)
