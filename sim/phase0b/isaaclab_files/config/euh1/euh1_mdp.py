# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""Custom MDP terms for the EUH-1 velocity task (torque-saturation logging)."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def torque_saturation_fraction(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    effort_limit: float,
    threshold_ratio: float = 0.9,
) -> torch.Tensor:
    """Fraction of the selected joints whose |applied torque| exceeds ``threshold_ratio * effort_limit``.

    Used as a weight~0 reward term purely for tensorboard logging via the reward manager
    (Phase 0b headline metric: how close do EUH-1 actuators run to saturation?).
    """
    asset = env.scene[asset_cfg.name]
    torques = asset.data.applied_torque[:, asset_cfg.joint_ids]
    return (torques.abs() > threshold_ratio * effort_limit).float().mean(dim=-1)
