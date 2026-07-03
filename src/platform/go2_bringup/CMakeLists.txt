cmake_minimum_required(VERSION 3.8)
project(go2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest QUIET)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(robot_common_interfaces REQUIRED)

# nlohmann/json — system package (MIT licence).
# Install: sudo apt install nlohmann-json3-dev
find_package(nlohmann_json REQUIRED)

# ── go2_hw_bridge ─────────────────────────────────────────────────────────────
# Central hardware adapter: SportModeState → odom/TF, cmd_vel → Sport API request
add_executable(go2_hw_bridge
  src/go2_hw_bridge.cpp
  src/sport_client.cpp
)

target_include_directories(go2_hw_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(go2_hw_bridge
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  unitree_api
  unitree_go
  robot_common_interfaces
  action_msgs
)

target_link_libraries(go2_hw_bridge
  nlohmann_json::nlohmann_json
)

install(TARGETS go2_hw_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Helper scripts (Python).  Installed as executables so they can be invoked
# via `ros2 run go2_bringup <script>` and from launch files.
install(PROGRAMS
  scripts/lowstate_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install directories ───────────────────────────────────────────────────────
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(test_sport_client
    test/test_sport_client.cpp
    src/sport_client.cpp
  )

  target_include_directories(test_sport_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_sport_client
    unitree_api
  )

  target_link_libraries(test_sport_client
    nlohmann_json::nlohmann_json
  )
endif()

ament_package()
