cmake_minimum_required(VERSION 3.8)
project(a2w_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
