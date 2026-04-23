#!/usr/bin/env python3
"""
dynamic_obstacle_tracker.py -- PLACEHOLDER, not active.

Future: subscribe to /lidar/points, cluster moving points between consecutive
frames, estimate each cluster velocity, and publish a smeared PointCloud2 on
/dynamic_obstacle_markers with projected future positions.

Input:  /lidar/points  (sensor_msgs/PointCloud2, frame: lidar_link)
Output: /dynamic_obstacle_markers  (sensor_msgs/PointCloud2, frame: odom)

Smearing: obstacle_centroid + velocity_vector * lookahead_time (default 1.5 s).
Add /dynamic_obstacle_markers as a second observation_source in the local
VoxelLayer once implemented.
"""
# TODO: implement
raise NotImplementedError("dynamic_obstacle_tracker is a placeholder.")
