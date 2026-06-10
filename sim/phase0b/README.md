# Phase 0b — does EUH-1's European actuation stack walk?

**Experiment:** take the stock Isaac Lab Unitree G1 velocity task (known-good morphology + training recipe, dimensionally ≈ EUH-1: 1.32 m / 35 kg / 23 DoF) and replace its actuator parameters with EUH-1's European-part values from `design/euh1-actuator-params.md`. Scale base mass −5 kg toward EUH-1's 30 kg target. Train, then measure how hard the policy pushes against the European torque limits.

This answers the #1 design risk (class-A torque margin) with data, before any CAD or purchasing.

## Runs

| Run | Config | Purpose | Est. wall-clock (1× RTX 4090) |
|---|---|---|---|
| R0 | Stock `Isaac-Velocity-Flat-G1-v0`, unmodified | pipeline smoke test + baseline metrics | ~1–2 h |
| R1 | EUH-1 actuator params, flat terrain | the core question | ~2–4 h |
| R2 | EUH-1 params, rough terrain + full DR + pushes | robustness margin | ~10–30 h |

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

## Cloud budget

4090 on Vast.ai/RunPod ≈ €0.40–0.60/h → R0+R1 ≈ €3, full suite incl. R2 ≈ €15–25.
