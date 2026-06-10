# Copyright (c) 2026, euro-humanoid Phase 0b.
# SPDX-License-Identifier: BSD-3-Clause

"""Standalone R1 evaluation: torque saturation, velocity tracking, fall rate.

Modeled on scripts/reinforcement_learning/rsl_rl/play.py but instruments the
step loop. Two phases:
  1. default Play command grid (lin_vel_x 0..1.0, lin_vel_y -0.5..0.5, ang_vel_z -1..1)
  2. forced 1.0 m/s forward command (the PASS-criterion gait)

Usage (inside the Isaac Lab container):
  ./isaaclab.sh -p /workspace/eval_r1.py --task Isaac-Velocity-Flat-EUH1-Play-v0 \
      --checkpoint /workspace/isaaclab/logs/rsl_rl/euh1_flat/2026-06-10_12-39-19/model_4999.pt \
      --num_envs 256 --steps 1600 --headless --out /workspace/r1_eval.json
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Phase 0b R1 eval.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Flat-EUH1-Play-v0")
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--steps", type=int, default=1600, help="steps per phase (0.02 s each)")
parser.add_argument("--warmup", type=int, default=100, help="steps excluded from torque/tracking stats")
parser.add_argument("--out", type=str, default="/workspace/r1_eval.json")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import json
import torch

from rsl_rl.runners import OnPolicyRunner

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import load_cfg_from_registry, parse_env_cfg

# joint classes under test (effort limits from EUH1 actuator cfg)
JOINT_CLASSES = {
    "class_a_80Nm": {
        "patterns": [".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_knee_joint"],
        "limit": 80.0,
    },
    "knee_80Nm": {"patterns": [".*_knee_joint"], "limit": 80.0},
    "hip_pitch_80Nm": {"patterns": [".*_hip_pitch_joint"], "limit": 80.0},
    "hip_roll_80Nm": {"patterns": [".*_hip_roll_joint"], "limit": 80.0},
    "ankle_pitch_48Nm": {"patterns": [".*_ankle_pitch_joint"], "limit": 48.0},
    "ankle_roll_20Nm": {
        "patterns": [".*_ankle_roll_joint"],
        "limit": 20.0,
    },
}
THRESHOLD = 0.9


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    env_cfg.seed = agent_cfg.seed

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    robot = env.unwrapped.scene["robot"]
    cmd_mgr = env.unwrapped.command_manager
    term_mgr = env.unwrapped.termination_manager
    device = env.unwrapped.device
    n_envs = env.unwrapped.num_envs
    dt = env.unwrapped.step_dt

    # resolve joint ids per class
    class_ids = {}
    for name, spec in JOINT_CLASSES.items():
        ids, jnames = robot.find_joints(spec["patterns"])
        class_ids[name] = torch.tensor(ids, dtype=torch.long, device=device)
        print(f"[EVAL] {name}: limit {spec['limit']} Nm, joints {jnames}")

    results = {
        "task": args_cli.task,
        "checkpoint": args_cli.checkpoint,
        "num_envs": n_envs,
        "steps_per_phase": args_cli.steps,
        "step_dt": dt,
        "warmup_steps": args_cli.warmup,
        "saturation_threshold": THRESHOLD,
        "phases": {},
    }

    def run_phase(label):
        obs = env.get_observations()
        sat_count = {k: 0 for k in JOINT_CLASSES}
        sat_total = {k: 0 for k in JOINT_CLASSES}
        peak_torque = {k: 0.0 for k in JOINT_CLASSES}
        falls = 0
        timeouts = 0
        lin_err_sq_sum = 0.0
        ang_err_sq_sum = 0.0
        vx_cmd_sum = 0.0
        vx_act_sum = 0.0
        track_samples = 0

        with torch.inference_mode():
            for i in range(args_cli.steps):
                actions = policy(obs)
                obs, _, dones, _ = env.step(actions)
                policy_nn.reset(dones)

                # falls: non-timeout terminations (counted over whole phase)
                falls += int(term_mgr.terminated.sum().item())
                timeouts += int(term_mgr.time_outs.sum().item())

                if i < args_cli.warmup:
                    continue

                # torque saturation on applied (post motor-model) torques
                tau = robot.data.applied_torque
                for k, spec in JOINT_CLASSES.items():
                    t = tau[:, class_ids[k]].abs()
                    sat_count[k] += int((t > THRESHOLD * spec["limit"]).sum().item())
                    sat_total[k] += t.numel()
                    peak_torque[k] = max(peak_torque[k], float(t.max().item()))

                # velocity tracking (base frame), all envs
                cmd = cmd_mgr.get_command("base_velocity")  # (N, 3): vx, vy, wz
                lin_err_sq = torch.sum((cmd[:, :2] - robot.data.root_lin_vel_b[:, :2]) ** 2, dim=1)
                ang_err_sq = (cmd[:, 2] - robot.data.root_ang_vel_b[:, 2]) ** 2
                lin_err_sq_sum += float(lin_err_sq.sum().item())
                ang_err_sq_sum += float(ang_err_sq.sum().item())
                vx_cmd_sum += float(cmd[:, 0].sum().item())
                vx_act_sum += float(robot.data.root_lin_vel_b[:, 0].sum().item())
                track_samples += n_envs

        episodes = falls + timeouts
        phase = {
            "saturation_fraction": {k: sat_count[k] / max(sat_total[k], 1) for k in JOINT_CLASSES},
            "peak_abs_torque_Nm": peak_torque,
            "effort_limits_Nm": {k: JOINT_CLASSES[k]["limit"] for k in JOINT_CLASSES},
            "falls": falls,
            "timeouts": timeouts,
            "episodes_ended": episodes,
            "fall_rate": falls / episodes if episodes > 0 else None,
            "sim_seconds": args_cli.steps * dt,
            "lin_vel_rms_error_mps": (lin_err_sq_sum / track_samples) ** 0.5,
            "ang_vel_rms_error_radps": (ang_err_sq_sum / track_samples) ** 0.5,
            "mean_cmd_vx_mps": vx_cmd_sum / track_samples,
            "mean_actual_vx_mps": vx_act_sum / track_samples,
        }
        results["phases"][label] = phase
        print(f"[EVAL] phase '{label}': {json.dumps(phase, indent=2)}")

    # phase 1: default Play command grid
    run_phase("default_commands")

    # phase 2: force 1.0 m/s straight-ahead command on all envs
    term = cmd_mgr.get_term("base_velocity")
    term.cfg.ranges.lin_vel_x = (1.0, 1.0)
    term.cfg.ranges.lin_vel_y = (0.0, 0.0)
    term.cfg.ranges.ang_vel_z = (0.0, 0.0)
    if hasattr(term.cfg.ranges, "heading") and term.cfg.ranges.heading is not None:
        term.cfg.ranges.heading = (0.0, 0.0)
    term.cfg.rel_standing_envs = 0.0
    all_ids = torch.arange(n_envs, device=device)
    term._resample_command(all_ids)
    run_phase("forced_1.0mps")

    with open(args_cli.out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"[EVAL] results written to {args_cli.out}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
