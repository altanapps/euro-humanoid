# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""EUH-1 rough-terrain velocity task: stock G1 task with EUH-1 actuators.

Differences vs. the stock G1 config (everything else inherited unchanged):

* ``scene.robot`` -> :obj:`EUH1_MINIMAL_CFG` (same G1 USD, DC-motor actuators per EUH-1 classes).
* ``events.add_base_mass`` re-enabled with a fixed -5.0 kg offset on ``torso_link``
  to approximate the EUH-1 30 kg mass target (stock G1 cfg disables this event).
* Two weight~0 reward terms log torque-saturation fractions to tensorboard.
  NOTE: RewardManager in Isaac Lab 2.3.2 skips terms whose weight is exactly 0.0,
  so a tiny weight (1e-8) is used; reward impact is negligible (<1e-9/step) and the
  logged ``Episode_Reward/torque_saturation_*`` curves are the fraction scaled by 1e-8*dt.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.rough_env_cfg import (
    G1Rewards,
    G1RoughEnvCfg,
    G1RoughEnvCfg_PLAY,
)

from . import euh1_mdp

##
# Pre-defined configs
##
from isaaclab_assets.robots.euh1 import EUH1_MINIMAL_CFG  # isort: skip


@configclass
class EUH1Rewards(G1Rewards):
    """G1 reward terms + torque-saturation logging terms (weight ~0, logging only)."""

    # Class A joints (hip pitch/roll, knee): effort_limit 80 Nm
    torque_saturation_class_a = RewTerm(
        func=euh1_mdp.torque_saturation_fraction,
        weight=1e-8,  # weight=0.0 terms are skipped by RewardManager in 2.3.2
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_knee_joint"],
            ),
            "effort_limit": 80.0,
            "threshold_ratio": 0.9,
        },
    )
    # Class C ankle-roll joints: effort_limit 20 Nm (the -50% vs G1 experiment)
    torque_saturation_ankle_roll = RewTerm(
        func=euh1_mdp.torque_saturation_fraction,
        weight=1e-8,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_roll_joint"]),
            "effort_limit": 20.0,
            "threshold_ratio": 0.9,
        },
    )


def apply_euh1_overrides(cfg):
    """Swap in the EUH-1 robot and the -5 kg torso mass offset (shared by rough/flat/play cfgs)."""
    # EUH-1 robot (same USD asset as G1, EUH-1 actuators)
    cfg.scene.robot = EUH1_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    # Fixed -5 kg on the torso to approximate the 30 kg EUH-1 target.
    # Stock G1 cfg sets events.add_base_mass = None, so we recreate the stock
    # velocity_env_cfg event term with a degenerate (fixed-offset) range.
    cfg.events.add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
            "mass_distribution_params": (-5.0, -5.0),
            "operation": "add",
        },
    )


@configclass
class EUH1RoughEnvCfg(G1RoughEnvCfg):
    rewards: EUH1Rewards = EUH1Rewards()

    def __post_init__(self):
        super().__post_init__()
        apply_euh1_overrides(self)


@configclass
class EUH1RoughEnvCfg_PLAY(G1RoughEnvCfg_PLAY):
    rewards: EUH1Rewards = EUH1Rewards()

    def __post_init__(self):
        super().__post_init__()
        apply_euh1_overrides(self)
