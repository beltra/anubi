# a2w_description

Unitree A2-W (Pro) model for the anubi stack.

- `meshes/`, `urdf/a2_upstream.urdf`: official Unitree A2 model from
  [unitree_ros](https://github.com/unitreerobotics/unitree_ros) `robots/a2_description`
  (commit 5994d4f), BSD-3-Clause, see `LICENSE.unitree_ros`.
- `urdf/a2w_body.urdf.xacro`: generated from the upstream URDF by
  `scripts/generate_a2w_xacro.py` (mesh paths, xacro leg joints, feet removed).
- `urdf/a2w.urdf.xacro`: entry point. Adds wheels and the sensor frames.
  `fixed_legs:=true` freezes the legs in the standing pose for the sim.

Unitree publishes no A2-W model. Wheel radius (0.095 m), wheel lateral offset
(0.06 m) and the standing pose (thigh 0.8, calf -1.5 rad, base ~0.50 m above the
floor) are estimates; correct them in `a2w.urdf.xacro` after measuring the robot.
