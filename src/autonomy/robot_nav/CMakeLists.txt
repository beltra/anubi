cmake_minimum_required(VERSION 3.8)
project(robot_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/distortion_corrector.py
  scripts/imu_stabilized_tf.py
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
