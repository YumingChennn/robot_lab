from robot_lab.assets import ISAACLAB_ASSETS_DATA_DIR

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg, DelayedPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##
from .utils import PaceDCMotorCfg

BIGREDDOG_PACE_ACTUATOR_CFG = PaceDCMotorCfg(
    joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
    saturation_effort=23.5,
    effort_limit=23.5,
    velocity_limit=30.0,
    stiffness={".*": 25.0},  # P gain in Nm/rad
    damping={".*": 0.5},  # D gain in Nm s/rad
    
    encoder_bias=[0.0] * 12,  # calf, encoder bias in radians
    max_delay=2,  # max delay in simulation steps

    armature={"FR_hip_joint": 0.0520, "FR_thigh_joint": 0.0482, "FR_calf_joint": 0.0451,
            "FL_hip_joint": 0.0555, "FL_thigh_joint": 0.0477, "FL_calf_joint": 0.0488,
            "RR_hip_joint": 0.0557, "RR_thigh_joint": 0.0493, "RR_calf_joint": 0.0489,
            "RL_hip_joint": 0.0587, "RL_thigh_joint": 0.0488, "RL_calf_joint": 0.0493},

    viscous_friction={"FR_hip_joint": 0.2607, "FR_thigh_joint": 0.2736, "FR_calf_joint": 0.2292,
                    "FL_hip_joint": 0.1521, "FL_thigh_joint": 0.2204, "FL_calf_joint": 0.2424,
                    "RR_hip_joint": 0.2572, "RR_thigh_joint": 0.2638, "RR_calf_joint": 0.3440,
                    "RL_hip_joint": 0.2522, "RL_thigh_joint": 0.2357, "RL_calf_joint": 0.2909},
    )

# BIGREDDOG_DELAYEDPD_ACTUATOR_CFG = DelayedPDActuatorCfg(
#     joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
#     effort_limit=23.5,
#     velocity_limit=30.0,
#     stiffness={".*": 25.0},  # P gain in Nm/rad
#     damping={".*": 0.5},  # D gain in Nm s/rad
#     friction={".*": 0},
#     armature={"FR_hip_joint": 0.0520, "FR_thigh_joint": 0.0482, "FR_calf_joint": 0.0451,
#             "FL_hip_joint": 0.0555, "FL_thigh_joint": 0.0477, "FL_calf_joint": 0.0488,
#             "RR_hip_joint": 0.0557, "RR_thigh_joint": 0.0493, "RR_calf_joint": 0.0489,
#             "RL_hip_joint": 0.0587, "RL_thigh_joint": 0.0488, "RL_calf_joint": 0.0493},
#     max_delay=10,  # max delay in simulation steps
#     )

# BIGREDDOG_DELAYEDPD_ACTUATOR_CFG = DelayedPDActuatorCfg(
#     joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
#     effort_limit=23.5,
#     velocity_limit=30.0,
#     stiffness={".*": 25.0},  # P gain in Nm/rad #25
#     damping={".*": 0.5},  # D gain in Nm s/rad #0.5
#     min_delay=0,  # min delay in simulation steps
#     max_delay=2,  # max delay in simulation steps
#     )

BIGREDDOG_SIMPLE_ACTUATOR_CFG = DCMotorCfg(
    joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
    effort_limit=23.5,
    saturation_effort=23.5,
    velocity_limit=30.0,
    stiffness= 25.0,
    damping= 0.5,
    friction=0.0,
    )

Big_reddog_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/bigreddog_new/urdf/big_reddog_0201.urdf",
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
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),
        joint_pos={
            "FR_hip_joint": -0.0,
            "FL_hip_joint": 0.0,
            "RR_hip_joint": -0.0,
            "RL_hip_joint": 0.0,

            "FR_thigh_joint": 0.6,
            "FL_thigh_joint": 0.6,
            "RR_thigh_joint": -0.6,
            "RL_thigh_joint": -0.6,

            "FR_calf_joint": -1.0,
            "FL_calf_joint": -1.0,
            "RR_calf_joint":  1.0,
            "RL_calf_joint":  1.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": BIGREDDOG_SIMPLE_ACTUATOR_CFG,
    },
)
"""Configuration of Big reddog using DC motor.
"""