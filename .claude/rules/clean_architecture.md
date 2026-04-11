---
description: Modular ROS2 Architecture Rules for this repository
---

# Modular ROS2 Architecture Rules

This repository is organized as a modular ROS2 workspace by package responsibility.
The default architecture is package-first, not a global `domain/application/infrastructure` split.

## Source Layout

```
src/
├── platform/         # Robot/platform-specific packages
├── core_interfaces/  # Shared message and API contracts
├── autonomy/         # Robot-agnostic autonomy stack
├── simulation/       # Simulation models/worlds/scenarios
└── vendors/          # Vendored third-party dependencies
```

## Core Rules

1. Keep package boundaries explicit.
2. Prefer simple ROS2 nodes and launch files over heavy abstraction.
3. Share contracts through `core_interfaces/*` packages, not through direct robot-specific coupling.
4. Keep simulation and real-robot contracts aligned (`docs/topic_map.md`).
5. Add complexity only when a concrete requirement demands it.

## Package Design Guidance

- For small/medium features, keep code package-local:
  - `launch/`, `config/`, `scripts/`, `src/`, `include/` as needed by package type.
- For reusable logic across robots, place it in robot-agnostic packages under `src/autonomy/`.
- For new message/service contracts, place them under `src/core_interfaces/`.

## Optional Advanced Layering (Inside a Package)

If a package grows significantly, you may apply internal layering inside that package:

```
<package>/
├── domain/          # Optional, pure logic
├── application/     # Optional orchestration
└── ros2/            # Nodes/adapters/launch-facing code
```

Use this only when it improves maintainability; it is not required repo-wide.

## Anti-Patterns

- Do not force global `src/domain`, `src/application`, `src/infrastructure` folders.
- Do not add DI/repository abstraction for one-off nodes.
- Do not duplicate TF ownership (`map->odom`, `odom->base_footprint`) across nodes.
- Do not break established topic/frame contracts without updating docs and launch wiring.

## Decision Checklist

Before introducing a new abstraction, answer yes to at least one:
- Is the logic reused by multiple packages?
- Are there at least two implementations to swap (real/sim/vendor)?
- Does testing or safety requirement need the extra indirection?

If all answers are no, keep the implementation simple.
