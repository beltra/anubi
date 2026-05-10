# Quadruped recovery package and BT wiring

Quadruped-aware recovery lives in `robot_nav_recovery` as loadable Nav2 behavior and BT plugins, while `robot_nav` remains the launch and configuration owner. The recovery tree uses local-costmap sector checks, guarded spin, guarded backup, guarded lateral step, and terminal diagnostics instead of blind generic recovery or automatic relocalization. This keeps recovery reusable across sim and real modes while preserving the normal Nav2 command and safety chain.
