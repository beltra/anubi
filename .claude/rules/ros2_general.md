---
description: ROS2 General Conventions aligned with this repository
---

# ROS2 General Conventions

## Package Naming

Use snake_case and descriptive names.

Naming convention in this repository:
- `go2_*` : Go2-specific packages
- `robot_*` : robot-agnostic/autonomy packages
- `sim_*` / `*_sim` : simulation packages and assets
- `unitree_*` : Unitree interfaces/contracts
- vendored packages keep upstream names

## Workspace Structure

```
src/
├── platform/
├── core_interfaces/
├── autonomy/
├── simulation/
└── vendors/
```

## Package Structure (Choose by package type)

### C++ package

```
package_name/
├── package.xml
├── CMakeLists.txt
├── include/package_name/
├── src/
├── launch/
└── config/
```

### Python package

```
package_name/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/package_name
├── package_name/
├── launch/
└── config/
```

### Interface package (msg/srv/action)

```
package_name/
├── package.xml
├── CMakeLists.txt
├── msg/
├── srv/
└── action/
```

### Data/asset package (worlds/models/urdf)

```
package_name/
├── package.xml
├── CMakeLists.txt
├── worlds/
├── models/
└── urdf/
```

## Launch and Parameters

- Prefer `get_package_share_directory()` and `PathJoinSubstitution`.
- Keep launch arguments explicit and documented.
- Keep parameter files under `config/`.

## QoS Practical Defaults

- Sensor streams: BEST_EFFORT, low depth.
- Control/commands: RELIABLE.
- Latched/static state: TRANSIENT_LOCAL where needed.

Use package-local QoS unless a shared profile package is explicitly introduced.

## Logging

- Use `debug` for high-rate details.
- Use `info` for lifecycle/state milestones.
- Use `warn/error` for degraded behavior and failures.
- Throttle repetitive logs on high-rate callbacks.
