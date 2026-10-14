cmake_minimum_required(VERSION 3.8)
project(a2w_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch policies
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/a2w_policy_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
