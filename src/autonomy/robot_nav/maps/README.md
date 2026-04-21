# Maps

Store slam_toolbox posegraph files here.

## Create a map

1. Launch mapping mode:

```bash
ros2 launch go2_bringup go2_bringup.launch.py slam_mode:=mapping
```

2. Drive the robot in the target area.

3. Save the posegraph:

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/ws/anubi/src/autonomy/robot_nav/maps/my_map'}"
```

This creates `my_map.posegraph` and `my_map.data`.

## Use a saved map

```bash
ros2 launch go2_bringup go2_bringup.launch.py \
  slam_mode:=localization \
  map:=/ws/anubi/src/autonomy/robot_nav/maps/my_map
```
