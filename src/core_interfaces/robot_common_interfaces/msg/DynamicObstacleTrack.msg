# Persistent dynamic obstacle track and its predicted future poses.

uint32 id
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
float32 speed
float32 yaw_rate
float32 confidence
geometry_msgs/Pose[] predicted_poses
builtin_interfaces/Time[] predicted_stamps
float32[] predicted_position_std
