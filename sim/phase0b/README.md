# Phase 0b — does EUH-1's European actuation stack walk?

**Experiment:** take the stock Isaac Lab Unitree G1 velocity task (known-good morphology + training recipe, dimensionally ≈ EUH-1: 1.32 m / 35 kg / 23 DoF) and replace its actuator parameters with EUH-1's European-part values from `design/euh1-actuator-params.md`. Scale base mass −5 kg toward EUH-1's 30 kg target. Train, then measure how hard the policy pushes against the European torque limits.

This answers the #1 design risk (class-A torque margin) with data, before any CAD or purchasing.

## Runs

| Run | Config | Purpose | Est. wall-clock (1× RTX 4090) |
|---|---|---|---|
| R0 | Stock `Isaac-Velocity-Flat-G1-v0`, unmodified | pipeline smoke test + baseline metrics | ~1–2 h |
| R1 | EUH-1 actuator params on G1 body, flat terrain | the core question | ~2–4 h |
| R2 | `Isaac-Velocity-Flat-EUH2-v0` — EUH-1 v1 **own body** (`sim/model/euh1.urdf`, 23 DoF, 31.88 kg, class-B ankle-roll), flat terrain | does our own morphology walk on EU actuators? | ~2–4 h |

## Metrics (logged per run, compared R1/R2 vs R0)

- Convergence: reward curve, iterations to first stable gait
- Velocity-tracking RMS (lin + ang) over command grid
- Fall rate (% terminations over 256 envs × 20 s)
- **Torque saturation fraction**: per joint, fraction of timesteps with |τ| > 0.9 × effort_limit — the headline number
- Mechanical cost of transport
- Push-recovery max impulse (R2 only)

## Pass / fail criteria

- **PASS:** R1 walks ≥ 1.0 m/s, fall rate < 2%, knee + ankle-roll saturation < 5% at nominal gait → class A/C specs confirmed, proceed to CAD (Phase 1).
- **MARGINAL:** saturation 5–20% on knees or ankle-roll → redesign those joints (maxon 270 Nm gearbox on knees / class-B ankle-roll), rerun.
- **FAIL:** no stable gait at EU limits → revisit mass budget and gear ratios before anything else.

## Files

- `euh1_g1_patch.py` — Isaac Lab actuator-config override (template; wire into a cloned `Isaac-Velocity-Flat-G1-v0` env config on the training box)
- `cloud_setup.sh` — turnkey provisioning for an Ubuntu 22.04 + RTX 4090 cloud box (Isaac Lab docker)
- `isaaclab_files/` — the actual Isaac Lab (2.3.2) source files deployed on the training box:
  - `robots/euh1.py` → `source/isaaclab_assets/isaaclab_assets/robots/euh1.py` (R1: G1 USD + EU `DCMotorCfg` actuators)
  - `robots/euh2.py` → `source/isaaclab_assets/isaaclab_assets/robots/euh2.py` (R2: own-body USD + EU actuators; ankle_roll promoted to class B, ankle_pitch kp/kd override)
  - `config/euh1/`, `config/euh2/` → `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/{euh1,euh2}/` (registers `Isaac-Velocity-{Flat,Rough}-EUH{1,2}[-Play]-v0`)
- The R2 USD was generated on the box with
  `./isaaclab.sh -p scripts/tools/convert_urdf.py /workspace/euro-humanoid/sim/model/euh1.urdf /workspace/assets/euh1/euh1.usd --headless`
  (no `--merge-joints` — the URDF has no fixed joints; primitive collision geometry and full URDF inertials import as-is). `robots/euh2.py` hardcodes that USD path.
- R2 train/eval on the box: `./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-EUH2-v0 --num_envs 4096 --max_iterations 6000 --headless` (resume: add `--resume`); eval headline metrics with `eval_r1.py` retargeted to `Isaac-Velocity-Flat-EUH2-Play-v0`. Do **not** render on the box — host driver 595.58.03 segfaults Isaac's RTX renderer; headless physics only.

## Cloud budget

4090 on Vast.ai/RunPod ≈ €0.40–0.60/h → R0+R1 ≈ €3, full suite incl. R2 ≈ €15–25.
