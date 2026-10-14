# SPDX-License-Identifier: Apache-2.0
"""Isaac Lab articulation config for the Unitree A2-W (Pro), derived from robot_lab's Go2W/B2W.

The URDF is exported from a2w_description (a2w.urdf.xacro fixed_legs:=false) by
export_urdf.sh; geometry, masses and leg/wheel effort limits come from the official
Unitree A2-W URDF. Wheel friction and damping are not published and are estimates.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

A2W_URDF_PATH = os.environ.get(
    "A2W_URDF_PATH",
    os.path.join(os.path.dirname(__file__), "description", "urdf", "a2w.urdf"),
)

# Standing pose shared with the Gazebo sim (a2w.urdf.xacro) and the policy default offset.
A2W_STAND_HIP = 0.0
A2W_STAND_THIGH = 0.8
A2W_STAND_CALF = -1.5

UNITREE_A2W_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=A2W_URDF_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.55),
        joint_pos={
            ".*_hip_joint": A2W_STAND_HIP,
            ".*_thigh_joint": A2W_STAND_THIGH,
            ".*_calf_joint": A2W_STAND_CALF,
            ".*_wheel_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # Official A2 limits: hip/thigh 120 Nm, 22 rad/s; calf 180 Nm, 14.67 rad/s.
        # Gains scaled between Go2W (kp 25 @ 23.5 Nm) and B2W (kp 160 @ 200/320 Nm).
        "hip": DCMotorCfg(
            joint_names_expr=[".*_hip_joint"],
            effort_limit=120.0,
            saturation_effort=120.0,
            velocity_limit=22.0,
            stiffness=100.0,
            damping=3.0,
            friction=0.0,
        ),
        "thigh": DCMotorCfg(
            joint_names_expr=[".*_thigh_joint"],
            effort_limit=120.0,
            saturation_effort=120.0,
            velocity_limit=22.0,
            stiffness=100.0,
            damping=3.0,
            friction=0.0,
        ),
        "calf": DCMotorCfg(
            joint_names_expr=[".*_calf_joint"],
            effort_limit=180.0,
            saturation_effort=180.0,
            velocity_limit=14.67,
            stiffness=100.0,
            damping=3.0,
            friction=0.0,
        ),
        # Official A2-W URDF: 24.5 Nm. Its 114.4 rad/s looks hand-entered; 60 rad/s at
        # r 0.0948 m covers the ~6 m/s spec top speed.
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_wheel_joint"],
            effort_limit_sim=24.5,
            velocity_limit_sim=60.0,
            stiffness=0.0,
            damping=1.5,
            friction=0.0,
        ),
    },
)
