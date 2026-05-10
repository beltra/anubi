# Safety arbiter command chain

The final motion path is Nav2 velocity smoothing, then `collision_monitor`, then the `velocity_limiter` safety arbiter, then the platform adapter. `collision_monitor` output is only collision-free, not hardware-safe; real production runs must use `/cmd_vel_safe` so watchdog, joystick override, tilt estop, and source arbitration remain the last gate before Sport Mode API commands. This preserves operator authority and makes disabling safety an explicit bench-only wrapper choice.
