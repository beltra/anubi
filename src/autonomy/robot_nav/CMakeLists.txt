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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav_profiles test/test_nav_profiles.py)
endif()

ament_package()
