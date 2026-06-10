# Phase 0b — R1 verdict (EUH-1 actuators on G1 morphology)

**Date:** 2026-06-10
**Run:** `logs/rsl_rl/euh1_flat/2026-06-10_12-39-19`, checkpoint `model_4999.pt` (5000 iters) → `r1_final.pt`
**Baseline:** `logs/rsl_rl/g1_flat/2026-06-10_11-52-26`, checkpoint `model_2999.pt` (3000 iters) → `r0_final.pt`
**Eval:** standalone instrumented play loop (`../eval_r1.py`), task `Isaac-Velocity-Flat-EUH1-Play-v0`, 256 envs, two phases of 32 s (1600 steps @ 50 Hz), first 2 s excluded from torque/tracking stats. Raw numbers: `r1_eval.json`.

## Torque saturation (fraction of (env, step, joint) samples with |τ| > 0.9 × limit)

| Joint group | Limit | Default commands (0–1 m/s grid) | Forced 1.0 m/s |
|---|---|---|---|
| Class A combined (hip pitch/roll + knee) | 80 Nm | 2.23% | 1.72% |
| — knee only | 80 Nm | **6.69%** | **5.15%** |
| — hip pitch | 80 Nm | 0.00% (peak 40.2 Nm) | 0.00% (peak 33.5 Nm) |
| — hip roll | 80 Nm | 0.00% (peak 49.4 Nm) | 0.00% (peak 22.1 Nm) |
| Ankle pitch (class B) | 48 Nm | 1.39% | 1.75% |
| **Ankle roll (class C)** | 20 Nm | **72.9%** | **77.3%** |

Knee and ankle-roll peak |τ| sit exactly at their limits (80.0 / 20.0 Nm) — both joints hit the DC-motor clamp; ankle-roll *lives* on it.

## Falls

0 non-timeout terminations across 768 completed episodes (256 + 512 over the two 32 s phases, 20 s episode length). **Fall rate 0.0%** (criterion < 2%).

## Velocity tracking

- Forced 1.0 m/s straight-ahead: mean achieved vₓ **0.962 m/s**, linear-velocity RMS error 0.128 m/s, yaw-rate RMS error 0.129 rad/s.
- Default command grid (mean cmd 0.49 m/s): linear RMS error 0.093 m/s, yaw RMS 0.144 rad/s, mean achieved vₓ ≈ mean commanded (0.489 vs 0.487).

## Training rewards (final, from train logs)

- R0 stock G1 (3000 iters): mean reward **28.93** (last 5: 29.04, 28.82, 28.47, 28.61, 28.93)
- R1 EUH-1 actuators (5000 iters): mean reward **21.69** (last 5: 22.82, 22.85, 21.96, 22.11, 21.69) — converged but plateaued ~25% below baseline, consistent with the ankle-roll authority deficit.

## Call: **MARGINAL**

R1 clears two of three PASS gates decisively — it walks at 1.0 m/s commanded (0.96 m/s achieved, tight tracking) with a 0% fall rate over 768 episodes — but fails the saturation gate on both joints the criterion names. Knees sit at 5.2–6.7%, just inside the 5–20% MARGINAL band. Ankle-roll is the real finding: at 73–77% saturation it is not marginal, it is pegged against the 20 Nm clamp for three-quarters of every gait cycle; the policy walks anyway because simulation tolerates indefinite peak-torque operation, but real hardware would have zero lateral control margin and cook the actuator thermally. Class A hips (0%) and class B ankle-pitch (<2%) specs are confirmed. Per the README prescription: upgrade ankle-roll from class C to class B (≥48 Nm) — and consider the maxon 270 Nm gearbox option for the knees while at it — then rerun R1 before any CAD on the ankle assembly.

## Files

- `r1_eval.json` — full eval numbers (both phases, per-joint)
- `r0_final.pt` / `r1_final.pt` — final policy checkpoints
- `tb/g1_flat/`, `tb/euh1_flat/` — tensorboard event files for both runs
- `../eval_r1.py` — the eval script (runs on the box via `./isaaclab.sh -p`)

## Known issue: no videos

Video rendering (`play.py --video`) is impossible on this Vast.ai box: Isaac Sim 5.1's RTX renderer segfaults at startup in `librtx.scenedb.plugin.so` under NVIDIA driver **595.58.03** — a [known 595.xx-branch incompatibility](https://forums.developer.nvidia.com/t/isaac-sim-5-1-crashes-on-startup-with-rtx-5060-ti-blackwell-sm-120-rtx-scenedb-plugin-crash/366252) ([also](https://github.com/isaac-sim/IsaacSim/issues/537)) whose only documented fix is a host driver downgrade (≤591/580), which a container tenant can't do. Vulkan itself enumerates the 4090 fine; headless training/eval (no rendering) is unaffected. Reproduced 4× incl. with multiGpu/DLSS/FabricSceneDelegate workarounds disabled; two pre-existing crash dumps show the same failure was hit during setup at 12:07 and 13:08. To get videos: rent a box whose host driver is ≤580.x and replay `r1_final.pt` / `r0_final.pt` there, or render locally.
