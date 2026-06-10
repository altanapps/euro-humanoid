# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""PPO runner configs for the EUH-2 velocity tasks (identical to G1, new experiment names)."""

from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.agents.rsl_rl_ppo_cfg import (
    G1FlatPPORunnerCfg,
    G1RoughPPORunnerCfg,
)


@configclass
class EUH2RoughPPORunnerCfg(G1RoughPPORunnerCfg):
    experiment_name = "euh2_rough"


@configclass
class EUH2FlatPPORunnerCfg(G1FlatPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "euh2_flat"
