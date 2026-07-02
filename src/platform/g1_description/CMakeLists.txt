cmake_minimum_required(VERSION 3.8)
project(g1_description)

find_package(ament_cmake REQUIRED)

# Install URDF/xacro, meshes and launch files into the package share dir so
# they resolve via package:// and ros2 launch.
install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
