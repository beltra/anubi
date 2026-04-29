cmake_minimum_required(VERSION 3.8)
project(robot_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_common_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(robot_nav)

add_library(dynamic_obstacles_common STATIC
  src/dynamic_obstacles_common.cpp
)
target_include_directories(dynamic_obstacles_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dynamic_obstacles_common Eigen3::Eigen)
ament_target_dependencies(
  dynamic_obstacles_common
  geometry_msgs
  rclcpp
  sensor_msgs
)

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
target_include_directories(lidar_level_frame_publisher PRIVATE include)

add_executable(dynamic_obstacle_preprocessor src/dynamic_obstacle_preprocessor.cpp)
target_link_libraries(dynamic_obstacle_preprocessor dynamic_obstacles_common)
ament_target_dependencies(
  dynamic_obstacle_preprocessor
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(dynamic_cluster_detector src/dynamic_cluster_detector.cpp)
target_link_libraries(dynamic_cluster_detector dynamic_obstacles_common)
ament_target_dependencies(
  dynamic_cluster_detector
  nav_msgs
  rclcpp
  robot_common_interfaces
  sensor_msgs
  tf2_ros
)

add_executable(dynamic_obstacle_tracker src/dynamic_obstacle_tracker.cpp)
target_link_libraries(dynamic_obstacle_tracker dynamic_obstacles_common)
ament_target_dependencies(
  dynamic_obstacle_tracker
  rclcpp
  robot_common_interfaces
)

add_executable(dynamic_obstacle_costmap_bridge src/dynamic_obstacle_costmap_bridge.cpp)
target_link_libraries(dynamic_obstacle_costmap_bridge dynamic_obstacles_common)
ament_target_dependencies(
  dynamic_obstacle_costmap_bridge
  rclcpp
  robot_common_interfaces
  sensor_msgs
  visualization_msgs
)

install(TARGETS
  dynamic_obstacle_preprocessor
  dynamic_cluster_detector
  dynamic_obstacle_tracker
  dynamic_obstacle_costmap_bridge
  distortion_corrector
  lidar_level_frame_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(PROGRAMS
  scripts/cloud_self_filter.py
  scripts/dynamic_cluster_detector.py
  scripts/dynamic_obstacle_costmap_bridge.py
  scripts/dynamic_obstacle_preprocessor.py
  scripts/dynamic_obstacle_tracker.py
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(test_dynamic_obstacles test/test_dynamic_obstacles.py)
  ament_add_gtest(test_dynamic_obstacles_cpp test/test_dynamic_obstacles_cpp.cpp)
  if(TARGET test_dynamic_obstacles_cpp)
    target_link_libraries(test_dynamic_obstacles_cpp dynamic_obstacles_common)
    target_include_directories(test_dynamic_obstacles_cpp PRIVATE include)
  endif()
endif()

ament_package()
