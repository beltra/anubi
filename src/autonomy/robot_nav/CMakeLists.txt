cmake_minimum_required(VERSION 3.8)
project(robot_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(robot_nav)

install(PROGRAMS
  scripts/cloud_self_filter.py
  scripts/distortion_corrector.py
  scripts/imu_stabilized_tf.py
  scripts/lidar_odom_relay.py
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
