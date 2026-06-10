# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""EUH-2 velocity-task configurations (EUH-1 v1 own body, EU actuators — Phase 0b R2)."""

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Velocity-Rough-EUH2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:EUH2RoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:EUH2RoughPPORunnerCfg",
    },
)


gym.register(
    id="Isaac-Velocity-Rough-EUH2-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:EUH2RoughEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:EUH2RoughPPORunnerCfg",
    },
)


gym.register(
    id="Isaac-Velocity-Flat-EUH2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:EUH2FlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:EUH2FlatPPORunnerCfg",
    },
)


gym.register(
    id="Isaac-Velocity-Flat-EUH2-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:EUH2FlatEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:EUH2FlatPPORunnerCfg",
    },
)
