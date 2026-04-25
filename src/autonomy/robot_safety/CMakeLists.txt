cmake_minimum_required(VERSION 3.8)
project(robot_safety)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/velocity_limiter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME velocity_limiter
)

install(PROGRAMS
  scripts/joystick_safety_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joystick_safety
)

install(PROGRAMS
  scripts/tilt_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tilt_monitor
)

install(PROGRAMS
  scripts/wireless_key_inspector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wireless_key_inspector
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_safety_nodes test/test_safety_nodes.py)
endif()

ament_package()
