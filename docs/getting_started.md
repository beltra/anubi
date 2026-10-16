# Getting started

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress
- A GPU helps: the simulated LiDARs use Gazebo's `gpu_lidar`
- For hardware: a PC on the robot's wired LAN (`192.168.123.x`)

## Install and build

The wrapper scripts source `/ws/anubi/install/setup.bash`, so clone into `/ws/anubi`
(or symlink it there).

```bash
sudo mkdir -p /ws && sudo chown $USER /ws
git clone https://github.com/beltra/anubi.git /ws/anubi
cd /ws/anubi
source /opt/ros/humble/setup.bash
```

Dependencies. `rosdep` covers most of them:

```bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y -r
```

If rosdep misses something, these are the main packages:

```bash
sudo apt install \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-spatio-temporal-voxel-layer ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-localization ros-humble-ros-gz \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf-transformations ros-humble-rviz2 \
  ros-humble-pcl-ros libpcl-dev libeigen3-dev libomp-dev nlohmann-json3-dev
```

Build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

To rebuild a single package, use `colcon build --packages-select <pkg>`. To include its
dependencies, use `--packages-up-to <pkg>`.

The Livox driver is skipped by default because its `package.xml` is generated. You only
need it on a real G1 (see [g1.md](g1.md)).

## Docker (optional)

`docker/Dockerfile.sim` has everything the simulation needs:

```bash
docker build -t anubi-sim:humble -f docker/Dockerfile.sim docker/
docker run -it --gpus all --network host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $PWD:/ws/anubi -w /ws/anubi anubi-sim:humble bash
```

Then build inside the container the same way as above. `docker/Dockerfile.real` is a
starting point for the G1 robot PC (CUDA torch, Unitree and Livox SDKs). It hasn't been
validated yet.

If Nav2 hangs while activating inside Docker, it's usually because stale FastDDS
shared memory filled the default 64 MB `/dev/shm`. Run `scripts/kill_ros_nodes.sh` to
clear it, or start the container with `--shm-size=1g`.

## First simulation run

```bash
scripts/run_go2_sim.sh
```

This starts Gazebo (office world), the robot, SLAM, Nav2 and RViz. SLAM starts after
20 s and Nav2 10 s after that, which gives the robot time to spawn and stand up. When
the map appears in RViz, click **2D Goal Pose** and drag a goal.

`scripts/run_g1_sim.sh` does the same for the G1. It starts headless by default; RViz
is the viewer.

The sim wrappers kill leftover ROS/Gazebo processes before they launch (`--no-clean`
turns that off). Anywhere else, run `scripts/kill_ros_nodes.sh` between runs.

## First run on a Go2

1. Connect the PC to the Go2 over Ethernet and give it a `192.168.123.x` address.
2. Install [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) and source
   its CycloneDDS environment:

   ```bash
   UNITREE_NET_IFACE=eth0 \
   UNITREE_ROS2_WS=~/unitree_ros2/cyclonedds_ws \
   source scripts/unitree_setup.sh
   ```

3. Check that the robot's topics are visible:

   ```bash
   ros2 topic hz /sportmodestate
   ros2 topic hz /utlidar/cloud
   ```

4. Start the stack in mapping mode:

   ```bash
   scripts/run_go2_real.sh
   ```

   Have the remote controller in your hand. Moving any stick takes over from Nav2 and
   holds the robot at zero velocity. See [usage.md](usage.md#safety-controls).

For the G1, follow [g1.md](g1.md).

## Next

- [usage.md](usage.md): mapping and localization, wrapper options, safety
- [architecture.md](architecture.md): how the packages fit together
