# Maps

Store slam_toolbox posegraph files here.

## Save current maps (DLIO 3D + Nav2 2D + slam posegraph)

From a running stack (sim or real), run:

```bash
/ws/anubi/scripts/save_current_maps.sh \
  --output-dir /ws/anubi/src/autonomy/robot_nav/maps/session_01
```

Outputs:

- `dlio_map.pcd` (3D DLIO map, if `save_pcd` service is available)
- `<map_basename>.yaml` + image (`.pgm`/`.png`) from Nav2 `/map`
- `<slam_basename>.posegraph` + `.data` from slam_toolbox

If `save_pcd` is missing, the script auto-starts a temporary `dlio_map_node` remapped to `/kf_cloud`.
Use `--dlio-keyframes-topic` if your DLIO keyframe topic is different.

Note: this repository localization flow loads slam_toolbox `.posegraph` maps, not Nav2 YAML maps.

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

### Simulation

```bash
/ws/anubi/scripts/run_go2_sim.sh localization \
  --map /ws/anubi/src/autonomy/robot_nav/maps/my_map.posegraph
```

### Real robot

```bash
/ws/anubi/scripts/run_go2_real.sh localization \
  --map /ws/anubi/src/autonomy/robot_nav/maps/my_map.posegraph
```

### Direct launch (equivalent)

```bash
ros2 launch go2_bringup go2_bringup.launch.py \
  slam_mode:=localization \
  map:=/ws/anubi/src/autonomy/robot_nav/maps/my_map.posegraph
```

You can also pass the path without extension if your workflow expects it, but using `.posegraph` explicitly is recommended.
