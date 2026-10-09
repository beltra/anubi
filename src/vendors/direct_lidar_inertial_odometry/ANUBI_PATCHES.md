# anubi-local patches to vendored DLIO

This vendored copy of `direct_lidar_inertial_odometry` carries intentional local
deltas from upstream. Keep this list current when editing the vendored source so the
divergence stays traceable (the workspace rule is to avoid *incidental* vendor edits;
these are deliberate and load-bearing).

## Integration deltas (anubi-specific, keep on rebase)

- `publish_tf` parameter (default true) gating the `odom->base_link` / `base_link->imu`
  / `base_link->lidar` broadcasts. Set false on the G1/Go2 real robot so the EKF owns
  `odom->base_footprint` and DLIO only publishes the `/lidar_odom` topic (no competing
  TF broadcaster). See `robot_lio` launch + `dlio_params_g1_real.yaml`.
- `SensorType::UNITREE` + the `time_stamp` point field, to deskew Unitree Go2 clouds
  whose per-point timestamp field is named `time_stamp` (Livox uses `timestamp`).

## Robustness fixes ported from the confirmed-working reference deploy

Ported from `talos-robotics-ai/Navigation` (`/home/beltra/unitree`, confirmed working
on the real G1). These are memory-safety / crash fixes and are topology-agnostic:

- **Async metric/debug threads made joinable, not detached.** They are now joined at
  the top of the next `callbackPointCloud` (and in the destructor). Detached, they
  raced the reassignment of `original_scan`/`current_scan` and the `push_back`s into
  the stats vectors -> use-after-free surfacing as glibc `free(): invalid next size`
  after ~50 s of running.
- **`path_ros` bounded and mutex-guarded.** Append+trim+publish (and the header writes)
  are under `path_mutex`, and the path is capped at `kMaxPathPoses` (2000). Unbounded
  growth + concurrent publish threads threw fastcdr `NotEnoughMemory` -> SIGABRT.
- **Non-finite point filter before the voxel grid.** NaN/Inf points from a bad
  IMU-integrated deskew are dropped before `pcl::VoxelGrid` (whose integer voxel-dim
  cast is UB on non-finite coords and can smash the heap). `removeNaN` misses Inf, so
  both are filtered explicitly.
- **`computeSpaciousness` off-by-one.** The range loop used `i <= size()` (one past the
  end); fixed to `i < size()` with an empty-cloud guard.

## Config override (in `robot_lio/config/dlio_params_g1_real.yaml`, not the vendored cfg)

- `map/waitUntilMove: false` -- the vendored `cfg/params.yaml` sets it true, which
  suppresses the deskewed cloud (the voxel/costmap input) until the robot has moved
  >= 0.1 m, leaving the map empty at a stationary mission start.
