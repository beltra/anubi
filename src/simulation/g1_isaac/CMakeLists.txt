cmake_minimum_required(VERSION 3.8)
project(g1_isaac)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/frame_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
