cmake_minimum_required(VERSION 3.8)
project(robot_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(robot_nav)

add_executable(distortion_corrector src/distortion_corrector.cpp)
ament_target_dependencies(
  distortion_corrector
  diagnostic_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(lidar_level_frame_publisher src/lidar_level_frame_publisher.cpp)
ament_target_dependencies(
  lidar_level_frame_publisher
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS
  distortion_corrector
  lidar_level_frame_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cloud_accumulator.py
  scripts/cloud_self_filter.py
  scripts/lidar_odom_relay.py
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  behavior_trees
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
