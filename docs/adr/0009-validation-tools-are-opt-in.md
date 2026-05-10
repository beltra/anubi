# Validation tools are opt-in

MPPI benchmarks and simulation safety regressions are validation entrypoints, not default bringup behavior. They may start scenario managers, bridge Gazebo world services, reset robot poses, and emit JSON reports, but those tools exist to compare profiles and catch regressions without changing normal sim or real launch defaults. This keeps routine bringup lightweight while preserving repeatable evidence for tuning and CI branches.
