# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""EUH-2 rough-terrain velocity task: EUH-1 v1 OWN body (R2), G1 training recipe.

This mirrors the stock ``G1RoughEnvCfg`` (which the R1 ``euh1`` package inherited)
but cannot inherit from it because all joint/link names differ — the EUH-1 URDF has
no ``_joint``/``_link`` suffixes, the torso link is ``torso`` (not ``torso_link``),
the waist DoF is ``waist_yaw`` (not ``torso_joint``), feet are ``.*_foot`` (not
``.*_ankle_roll_link``), there are no finger joints, and arms end in ``wrist_roll``
(not ``elbow_roll``). All reward weights / DR settings are kept identical to the R1
template; the only intentional differences:

* ``events.add_base_mass`` stays ``None`` (stock G1 behavior). R1's fixed -5 kg torso
  offset was a surrogate-mass correction; the own body is already 31.88 kg.
* Saturation metric joints: class A (80 Nm) on hip_roll/hip_pitch/knee_pitch;
  ankle_roll is now class B, so it is checked against 0.9 * 48 Nm (was 0.9 * 20).

Saturation terms use weight 1e-8 because RewardManager in Isaac Lab 2.3.2 skips
terms whose weight is exactly 0.0; reward impact is negligible (<1e-9/step) and the
logged ``Episode_Reward/torque_saturation_*`` curves are the fraction scaled by 1e-8*dt.

Install on the training box as
``source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/euh2/``.
"""

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

from . import euh2_mdp

##
# Pre-defined configs
##
from isaaclab_assets.robots.euh2 import EUH2_CFG  # isort: skip


@configclass
class EUH2Rewards(RewardsCfg):
    """G1-recipe reward terms retargeted to EUH-1 v1 names, + saturation logging."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, weight=2.0, params={"command_name": "base_velocity", "std": 0.5}
    )
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "threshold": 0.4,
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
        },
    )

    # Penalize ankle joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_pitch", ".*_ankle_roll"])},
    )
    # Penalize deviation from default of the joints that are not essential for locomotion
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw", ".*_hip_roll"])},
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_shoulder_pitch",
                    ".*_shoulder_roll",
                    ".*_shoulder_yaw",
                    ".*_elbow_pitch",
                    ".*_wrist_roll",
                ],
            )
        },
    )
    joint_deviation_torso = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names="waist_yaw")},
    )

    # --- Phase 0b saturation metrics (weight ~0, logging only) ---
    # Class A joints (hip pitch/roll, knee): effort_limit 80 Nm
    torque_saturation_class_a = RewTerm(
        func=euh2_mdp.torque_saturation_fraction,
        weight=1e-8,  # weight=0.0 terms are skipped by RewardManager in 2.3.2
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[".*_hip_pitch", ".*_hip_roll", ".*_knee_pitch"],
            ),
            "effort_limit": 80.0,
            "threshold_ratio": 0.9,
        },
    )
    # Ankle-roll joints, now class B: effort_limit 48 Nm (v1 promoted C -> B)
    torque_saturation_ankle_roll = RewTerm(
        func=euh2_mdp.torque_saturation_fraction,
        weight=1e-8,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_roll"]),
            "effort_limit": 48.0,
            "threshold_ratio": 0.9,
        },
    )


@configclass
class EUH2RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: EUH2Rewards = EUH2Rewards()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # Scene
        self.scene.robot = EUH2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/torso"

        # Randomization (G1 recipe; add_base_mass stays None — own body is already 31.88 kg)
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["torso"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.base_com = None

        # Rewards
        self.rewards.lin_vel_z_l2.weight = 0.0
        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.dof_acc_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=[".*_hip_.*", ".*_knee_pitch"]
        )
        self.rewards.dof_torques_l2.weight = -1.5e-7
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=[".*_hip_.*", ".*_knee_pitch", ".*_ankle_.*"]
        )

        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

        # terminations: undesired contact on torso or pelvis
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["torso", "pelvis"]


@configclass
class EUH2RoughEnvCfg_PLAY(EUH2RoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (1.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None
