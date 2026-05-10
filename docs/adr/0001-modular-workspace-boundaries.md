# Modular ROS workspace boundaries

The workspace is organized around platform adapters, robot-agnostic autonomy, shared interfaces, simulation packages, and vendored dependencies. This keeps Go2-specific boot, Sport Mode API, and hardware quirks out of reusable navigation, safety, LIO, and recovery packages while still allowing simulation to mirror the real robot contracts. Future features should extend the existing package boundary that owns the behavior instead of adding parallel launch stacks or robot-specific shortcuts.
