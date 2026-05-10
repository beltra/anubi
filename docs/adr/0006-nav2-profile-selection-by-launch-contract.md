# Nav2 profile selection by launch contract

Nav2 parameter selection is driven by `use_stvl`, `nav_profile`, and optional `params_file` rather than hardcoding one YAML path. `default_safe` remains the normal STVL-backed profile, `narrow_gap_1p0m` and `forward_static` remain opt-in profiles, and route_server experiments are launched separately without changing the command chain. This lets tuning and benchmarking compare profiles while preserving a predictable default for sim and real wrappers.

Real-robot launch files that include both `ekf.launch.py` and `nav2.launch.py` pass an explicit empty `params_file` to Nav2. This prevents EKF's generic `params_file` launch argument from leaking into Nav2 and bypassing profile selection.
