# Sim-real launch and topic parity

Simulation and real Go2 bringup share wrapper arguments and core topic names wherever the behavior is conceptually the same. The current `feat/sim2real` contract keeps mode, map, Nav2 profile, LIO, STVL, route_server, robot name, LiDAR correction IMU, and nav-delay controls aligned, while preserving real-only safety and LiDAR relay inputs. This decision makes sim validation meaningful for real hardware work and pushes intentional differences into documented wrapper options rather than hidden launch forks.
