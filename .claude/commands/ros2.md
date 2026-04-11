# ROS2 Common Commands (Humble + Fortress)

These commands target this workspace environment:
- ROS 2 Humble
- Gazebo Fortress

## Build System (colcon)

| Action | Command | Description |
| --- | --- | --- |
| **Build Workspace** | `colcon build --symlink-install` | Build all packages with symlinks (Python edits do not require reinstall) |
| **Build Package** | `colcon build --packages-select <pkg_name>` | Build only one package |
| **Build Up To** | `colcon build --packages-up-to <pkg_name>` | Build a package and its dependencies |
| **Clean Build** | `rm -rf build/ install/ log/` | Remove generated artifacts (use with caution) |
| **Test** | `colcon test` | Run tests (when available) |
| **Test Output** | `colcon test-result --all` | Show test results |

## Environment

| Action | Command | Description |
| --- | --- | --- |
| **Source Overlay** | `source install/setup.bash` | Source the workspace overlay |
| **Check Env** | `printenv | grep ROS` | Check ROS environment variables |

## Introspection (ros2)

### Nodes

| Command | Description |
| --- | --- |
| `ros2 node list` | List all running nodes |
| `ros2 node info <node_name>` | Show publishers/subscribers/services of a node |

### Topics

| Command | Description |
| --- | --- |
| `ros2 topic list` | List active topics |
| `ros2 topic echo <topic>` | Print messages from a topic |
| `ros2 topic info <topic>` | Show type and pub/sub count |
| `ros2 topic pub <topic> <type> <args>` | Publish a message manually |
| `ros2 topic hz <topic>` | Check publish frequency |
| `ros2 topic bw <topic>` | Check topic bandwidth |

### Services

| Command | Description |
| --- | --- |
| `ros2 service list` | List available services |
| `ros2 service call <service> <type> <args>` | Call a service manually |
| `ros2 service type <service>` | Show service type |

### Actions

| Command | Description |
| --- | --- |
| `ros2 action list` | List available actions |
| `ros2 action send_goal <action> <type> <goal>` | Send an action goal |

### Parameters

| Command | Description |
| --- | --- |
| `ros2 param list` | List node parameters |
| `ros2 param get <node> <param>` | Get parameter value |
| `ros2 param set <node> <param> <value>` | Set parameter value |
| `ros2 param dump <node>` | Dump parameters to YAML |

## Debugging Tools

| Tool | Command | Description |
| --- | --- | --- |
| **RQT Graph** | `rqt_graph` | Visualize node/topic graph |
| **RQT Console** | `rqt_console` | Filter and inspect logs |
| **TF Tree** | `ros2 run tf2_tools view_frames` | Generate TF tree output |
| **Doctor** | `ros2 doctor` | Check ROS environment issues |
