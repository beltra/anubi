cmake_minimum_required(VERSION 3.8)
project(robot_nav_recovery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  behaviortree_cpp_v3
  diagnostic_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_behaviors
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(include)

add_library(robot_nav_recovery_core SHARED
  src/costmap_sector_analyzer.cpp
  src/motion_stuck_detector.cpp
)
ament_target_dependencies(robot_nav_recovery_core ${dependencies})

add_library(robot_nav_recovery_behaviors SHARED
  src/recovery_behaviors.cpp
)
target_link_libraries(robot_nav_recovery_behaviors robot_nav_recovery_core)
ament_target_dependencies(robot_nav_recovery_behaviors ${dependencies})

add_library(robot_nav_recovery_bt_nodes SHARED
  src/recovery_bt_nodes.cpp
)
target_compile_definitions(robot_nav_recovery_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(robot_nav_recovery_bt_nodes robot_nav_recovery_core)
ament_target_dependencies(robot_nav_recovery_bt_nodes ${dependencies})

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS
  robot_nav_recovery_core
  robot_nav_recovery_behaviors
  robot_nav_recovery_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  behavior_plugin.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_costmap_sector_analyzer test/test_costmap_sector_analyzer.cpp)
  target_link_libraries(test_costmap_sector_analyzer robot_nav_recovery_core)
  ament_target_dependencies(test_costmap_sector_analyzer ${dependencies})

  ament_add_gtest(test_motion_stuck_detector test/test_motion_stuck_detector.cpp)
  target_link_libraries(test_motion_stuck_detector robot_nav_recovery_core)
  ament_target_dependencies(test_motion_stuck_detector ${dependencies})

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  target_link_libraries(test_plugin_loading robot_nav_recovery_core)
  target_link_libraries(test_plugin_loading ${CMAKE_DL_LIBS})
  ament_target_dependencies(test_plugin_loading ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(
  robot_nav_recovery_core
  robot_nav_recovery_behaviors
  robot_nav_recovery_bt_nodes
)
ament_export_dependencies(${dependencies})

ament_package()
