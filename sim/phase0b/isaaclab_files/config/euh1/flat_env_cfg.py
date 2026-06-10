# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""EUH-1 flat-terrain velocity task: stock G1 flat task with EUH-1 actuators.

Inherits all flat-terrain tweaks from the stock ``G1FlatEnvCfg`` and applies the
EUH-1 robot/mass overrides on top (see ``rough_env_cfg.apply_euh1_overrides``).
"""

from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.flat_env_cfg import (
    G1FlatEnvCfg,
    G1FlatEnvCfg_PLAY,
)

from .rough_env_cfg import EUH1Rewards, apply_euh1_overrides


@configclass
class EUH1FlatEnvCfg(G1FlatEnvCfg):
    rewards: EUH1Rewards = EUH1Rewards()

    def __post_init__(self):
        # stock G1 rough + flat post-init
        super().__post_init__()
        # EUH-1 robot + -5 kg torso mass
        apply_euh1_overrides(self)


@configclass
class EUH1FlatEnvCfg_PLAY(G1FlatEnvCfg_PLAY):
    rewards: EUH1Rewards = EUH1Rewards()

    def __post_init__(self):
        super().__post_init__()
        apply_euh1_overrides(self)
