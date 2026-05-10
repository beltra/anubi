# Gravity-aligned corrected LiDAR cloud

Anubi treats `/lidar/points_undistorted` as the single corrected cloud consumed by SLAM, Nav2, collision monitoring, and dynamic-obstacle work. The runtime processor is the C++ `distortion_corrector`; it preserves the Python reference behavior while applying the inverse LiDAR mount rotation, publishing in `lidar_stabilized` when state is available, and dropping rather than passing raw clouds through when required state is stale. This prevents downstream consumers from confusing raw tilted clouds with corrected perception.
