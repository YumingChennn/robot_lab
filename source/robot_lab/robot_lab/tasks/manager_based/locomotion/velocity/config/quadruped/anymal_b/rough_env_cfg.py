# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from source.robot_lab.robot_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg_r import LocomotionVelocityRoughEnvCfg

from isaaclab.utils import configclass

##
# Pre-defined configs
##
from robot_lab.assets.anymal import ANYMAL_B_CFG  # isort: skip


@configclass
class AnymalBRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    base_link_name = "base"
    foot_link_name = ".*FOOT"

    joint_names = [
        "RF_HAA", "RF_HFE", "RF_KFE", # FR leg  -> RF
        "LF_HAA", "LF_HFE", "LF_KFE", # FL leg  -> LF
        "RH_HAA", "RH_HFE", "RH_KFE", # RR leg  -> RH
        "LH_HAA", "LH_HFE", "LH_KFE", # RL leg  -> LH
    ]

    HAA_joint_names = [
        "RF_HAA", "LF_HAA", "RH_HAA", "LH_HAA",
    ]

    HFE_joint_names = [
        "RF_HFE", "LF_HFE", "RH_HFE", "LH_HFE",
    ]

    KFE_joint_names = [
        "RF_KFE", "LF_KFE", "RH_KFE", "LH_KFE",
    ]

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # ------------------------------Sence------------------------------
        self.scene.robot = ANYMAL_B_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        self.scene.height_scanner_base.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name

        # ------------------------------Observations------------------------------
        self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None
        self.observations.policy.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.policy.joint_vel.params["asset_cfg"].joint_names = self.joint_names

        # ------------------------------Actions------------------------------
        # reduce action scale
        self.actions.joint_pos.scale = {".*_HAA": 0.125, "^(?!.*_HAA).*": 0.25}
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}
        self.actions.joint_pos.joint_names = self.joint_names

        # ------------------------------Events------------------------------
        self.events.randomize_reset_base.params = {
            "pose_range": {
                "x": (-1.0, 1.0),
                "y": (-1.0, 1.0),
                "z": (0.0, 0.0),
                "roll": (-0.3, 0.3),
                "pitch": (-0.3, 0.3),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.2, 0.2),
                "roll": (-0.05, 0.05),
                "pitch": (-0.05, 0.05),
                "yaw": (-0.0, 0.0),
            },
        }
        self.events.randomize_rigid_body_mass_base.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_rigid_body_mass_others.params["asset_cfg"].body_names = [
            f"^(?!.*{self.base_link_name}).*"
        ]
        self.events.randomize_com_positions.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_apply_external_force_torque.params["asset_cfg"].body_names = [self.base_link_name]

        # ------------------------------Rewards------------------------------
        # General
        # Action penalties
        self.rewards.action_rate_l2.weight = -0.02

        self.rewards.base_height_l2.weight = -5.0
        self.rewards.base_height_l2.params["target_height"] = 0.427
        self.rewards.base_height_l2.params["asset_cfg"].body_names = [self.base_link_name]
    
        self.rewards.body_lin_acc_l2.weight = 0
        self.rewards.body_lin_acc_l2.params["asset_cfg"].body_names = [self.base_link_name]

        # Others
        self.rewards.feet_air_time.weight = 5.0
        self.rewards.feet_air_time.params["threshold"] = 0.5
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_air_time_variance.weight = -2.5
        self.rewards.feet_air_time_variance.params["sensor_cfg"].body_names = [self.foot_link_name]

        self.rewards.feet_slide.weight = -0.05
        self.rewards.feet_slide.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.stand_still.weight = -2.0 # -2.0

        self.rewards.feet_height_body.weight = 0.0 # Disabled to avoid conflict with base_height_l2
        self.rewards.feet_height_body.params["target_height"] = -0.36
        self.rewards.feet_height_body.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height.weight = -0.2
        self.rewards.feet_height.params["target_height"] = 0.05
        self.rewards.feet_height.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.contact_forces.weight = -1.5e-4 #-1.5e-4
        self.rewards.contact_forces.params["sensor_cfg"].body_names = [self.foot_link_name]

        # Root penalties
        self.rewards.lin_vel_z_l2.weight = -2.0
        self.rewards.ang_vel_xy_l2.weight = -0.02

        # Velocity-tracking rewards
        self.rewards.track_lin_vel_xy_exp.weight = 6.0 # 3.0
        self.rewards.track_ang_vel_z_exp.weight = 3.0 # 1.5

        # Contact sensor
        self.rewards.undesired_contacts.weight = -0.5
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]

        # Joint penalties
        self.rewards.joint_torques_l2.weight = -2.5e-5
        self.rewards.joint_vel_l2.weight = 0
        self.rewards.joint_acc_l2.weight = -1e-8
        self.rewards.joint_power.weight = -2e-5
        self.rewards.flat_orientation_l2.weight = -5.0

        # self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_l1", 0, [""])
        self.rewards.joint_pos_limits.weight = -5.0
        self.rewards.joint_vel_limits.weight = 0

        # self.rewards.joint_pos_penalty.weight = -0.5 # Reduced from -1.0 to encourage movement
        self.rewards.HAA_joint_pos_penalty.weight = -0.1
        self.rewards.HAA_joint_pos_penalty.params["asset_cfg"].joint_names = self.HAA_joint_names
        self.rewards.HFE_joint_pos_penalty.weight = -0.1
        self.rewards.HFE_joint_pos_penalty.params["asset_cfg"].joint_names = self.HFE_joint_names
        self.rewards.KFE_joint_pos_penalty.weight = -0.1
        self.rewards.KFE_joint_pos_penalty.params["asset_cfg"].joint_names = self.KFE_joint_names
        # joint_mirror: For trot gait, use diagonal pairs
        # Diagonal legs have similar joint angles when moving together
        self.rewards.joint_mirror.weight = 0
        self.rewards.joint_mirror.params["mirror_joints"] = [
            ["LF_(HAA|HFE|KFE).*", "RH_(HAA|HFE|KFE).*"],  # 左前 <-> 右後 (diagonal pair 1)
            ["RF_(HAA|HFE|KFE).*", "LH_(HAA|HFE|KFE).*"],  # 右前 <-> 左後 (diagonal pair 2)
        ]

        self.rewards.feet_contact.weight = 0
        self.rewards.feet_contact.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact_without_cmd.weight = 0
        self.rewards.feet_contact_without_cmd.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_stumble.weight = 0
        self.rewards.feet_stumble.params["sensor_cfg"].body_names = [self.foot_link_name]
        
        # feet_height_body: Penalize feet too high relative to body (prevent excessive lifting)

        # feet_gait: Encourage trot gait with diagonal pairs
        # Diagonal pairs move together in trot gait
        self.rewards.feet_gait.weight = 0.0
        self.rewards.feet_gait.params["synced_feet_pair_names"] = (
            ("LF_FOOT", "RH_FOOT"),  # 左前 + 右後 (diagonal pair 1)
            ("RF_FOOT", "LH_FOOT")   # 右前 + 左後 (diagonal pair 2)
        )
        # upward: Encourage upright posture (projected_gravity_b[:, 2] ≈ -1 when upright)
        self.rewards.upward.weight = 0.08  # Reduced from 1.0: avoid dominating velocity tracking

        # If the weight of rewards is 0, set rewards to None
        if self.__class__.__name__ == "AnymalDRoughEnvCfg":
            self.disable_zero_weight_rewards()

        # ------------------------------Terminations------------------------------
        self.terminations.illegal_contact.params["sensor_cfg"].body_names = [self.base_link_name]
        # self.terminations.illegal_contact.params["sensor_cfg"].body_names = [self.base_link_name, ".*_HIP"]
        # self.terminations.illegal_contact = None

        # ------------------------------Curriculums------------------------------
        # self.curriculum.command_levels_lin_vel.params["range_multiplier"] = (0.2, 1.0)
        # self.curriculum.command_levels_ang_vel.params["range_multiplier"] = (0.2, 1.0)
        self.curriculum.command_levels_lin_vel = None
        self.curriculum.command_levels_ang_vel = None

        # ------------------------------Commands------------------------------
        self.commands.base_velocity.ranges.lin_vel_x = (-1.5, 1.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.5, 1.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.8, 0.8)
