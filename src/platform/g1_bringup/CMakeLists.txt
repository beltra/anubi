cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

# Build type matches go2_bringup and the repo-wide convention (ament_cmake).
# g1_bringup ships only Python nodes, so -- like robot_safety -- we install the
# scripts as runnable executables under lib/${PROJECT_NAME} via install(PROGRAMS)
# rather than using ament_python.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# -- Python nodes --------------------------------------------------------------
# Installed (and RENAMEd, dropping the _node.py suffix) so they are runnable as:
#   ros2 run g1_bringup cmd_vel_to_amo
#   ros2 run g1_bringup cmd_vel_to_sonic
#   ros2 run g1_bringup imu_rescale
#   ros2 run g1_bringup g1_state_bridge
install(PROGRAMS
  scripts/cmd_vel_to_amo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_to_amo
)
install(PROGRAMS
  scripts/cmd_vel_to_sonic_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_to_sonic
)
install(PROGRAMS
  scripts/imu_rescale_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_rescale
)
install(PROGRAMS
  scripts/g1_state_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME g1_state_bridge
)

# -- Install directories -------------------------------------------------------
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
