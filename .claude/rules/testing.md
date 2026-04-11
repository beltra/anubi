---
description: Testing strategy for this repository (integration-led)
---

# ROS2 Testing Strategy

This repository is currently integration-led.
There is no mandatory global unit-test pyramid across all packages.

## Current State

- Primary validation path: launch/integration behavior in sim and hardware bringup.
- Some vendored packages may contain their own tests.
- New tests should be added incrementally where they provide direct value.

## Recommended Test Levels

### 1. Smoke launch tests (highest ROI)

Validate that key launch files start and core topics/TF are available.

Examples:
- `robot_sim sim_bringup.launch.py`
- `go2_bringup go2_bringup.launch.py`

### 2. Node integration tests

For safety-critical or bridge nodes, validate:
- topic I/O contracts
- timeout/watchdog behavior
- parameter handling

### 3. Unit tests (optional)

Add unit tests when pure logic exists and can be tested without ROS graph.
Do not force unit-test layering where code is mostly ROS glue.

## Commands

```bash
# Build
colcon build --symlink-install

# Run available tests
colcon test
colcon test-result --all
```

## Minimum Validation for New Changes

Before merging changes that affect runtime behavior:
1. Build affected packages.
2. Run at least one relevant launch smoke scenario.
3. Verify topic/TF contract impact against docs/topic_map.md.

## Anti-Patterns

- Do not block changes waiting for a full repo-wide test framework migration.
- Do not add heavy mocking/DI layers only to satisfy synthetic unit tests.
- Do not skip integration checks when modifying launch, TF, or safety behavior.
