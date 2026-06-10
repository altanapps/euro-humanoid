# Simulation pipeline — Isaac Lab (June 2026)

**Version call:** build on **Isaac Lab 2.3.x stable** (Isaac Sim 5.x). Isaac Lab 3.0 Beta / Newton physics is out but not the safe base yet. Export **both URDF and MJCF from day one** — MuJoCo sim2sim is the gate before any hardware.

## Asset pipeline

CAD (Onshape) → [Rhoban/onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) → URDF + MJCF → Isaac Lab `UrdfConverter` (`scripts/tools/convert_urdf.py`) → instanceable USD → `ArticulationCfg`.

Pitfalls that burn time:
- Override importer joint-drive defaults in `ArticulationCfg.actuators`; never trust importer gains.
- Replace foot/hand collision meshes with primitive boxes/capsules; importer always convex-approximates.
- `import_inertia_tensor=True`; CAD inertias wrong unless per-part densities set (motors, PCBs, battery) — weigh real subassemblies and patch.
- Isaac Sim 5.1 changed fixed-joint merging — re-check total mass/DoF after import.
- Self-collision off initially. Drive every joint ±limit in GUI before training.

## Actuator modelling

Use `DCMotorCfg` (torque-speed curve) or Berkeley Humanoid's `IdentifiedActuatorCfg` (DCMotor + identified friction). Parameters per joint, from EU part datasheets:

| Param | Source |
|---|---|
| `effort_limit` / `saturation_effort` | datasheet (peak/stall, post-gearbox) |
| `velocity_limit` | no-load speed @ 48 V |
| `armature` | rotor inertia × gear ratio² |
| `friction` | bench sysID (back-drive test) — until hardware exists, borrow Berkeley values scaled |
| `stiffness`/`damping` | match the gains the MD80/Circulo will actually run |

Reference values: Berkeley Humanoid hip kp 15 kd 1.5 effort 30 Nm armature 7.6e-4; G1 configs kp 40–150. Note: stock G1 task convergence is coupled to its exact asset — expect retuning.

## Locomotion training

- Clone `isaaclab_tasks/.../velocity/config/g1/` → retarget joint names, default pose, contact bodies.
- rsl_rl PPO, 4096 envs, MLP [512,256,128]; flat ~3k iters (≈1–4 h on one RTX 4090), rough+DR 15–20k iters (≈4–30 h).
- Rewards: track_lin_vel_xy (1.0), track_ang_vel_z (0.5–1.0), feet_air_time (0.25–1.0); penalties: torques, action_rate, flat_orientation, pose deviation, undesired contacts, termination (-200).
- DR: friction U(0.5–2.0), base mass ±1–5 kg, kp/kd ×U(0.75,1.25), push events every ~10 s, obs noise, 20–40 ms control delay.
- Policy 50 Hz / PD 200 Hz+; obs = projected gravity + ang vel + joint pos/vel + last action (no base lin vel).

## Eval suite

**Locomotion** (fixed command grid, ≥256 envs × 20 s, flat/rough × DR-on/off):
velocity-tracking RMS (lin+ang) · fall rate (% terminations) · mechanical cost of transport Σ|τ·q̇|/(mgd) · push-recovery impulse sweep (4–8 directions, max recoverable impulse) · gait quality (air time, foot slip, action smoothness).

**Manipulation** ([Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena) as harness, ≥50 seeds/task): grasp success rate · pick-and-place success · time-to-completion · drop rate; object pose ±5–10 cm + friction randomized. Loco-manipulation: walk-to-table → pick → carry → place (Isaac Lab 2.3 G1 loco-manip env as template; RL legs + IK upper body).

**Sim2sim gate:** export TorchScript → replay in plain MuJoCo with our MJCF. A policy that dies in MuJoCo dies on the robot. The Isaac↔MuJoCo metric delta is the best pre-hardware proxy for the sim2real gap.

## Repos to fork/study

1. https://github.com/isaac-sim/IsaacLab (`./isaaclab.sh --new` scaffold)
2. https://github.com/HybridRobotics/isaac_berkeley_humanoid — canonical custom-humanoid-with-custom-actuators repo
3. https://github.com/fan-ziqi/robot_lab — cleanest "add your own robot" patterns
4. https://github.com/unitreerobotics/unitree_rl_lab — production configs + sim2sim/sim2real chain
5. https://github.com/Rhoban/onshape-to-robot
6. https://github.com/isaac-sim/IsaacLab-Arena — eval harness
7. Hedges: https://github.com/mujocolab/mjlab (Isaac-Lab-style API on MuJoCo-Warp), mujoco_playground, kscalelabs/ksim
8. Later: https://github.com/NVlabs/HOVER (whole-body tracking control)

## Ordered steps → "walking policy + eval report"

1. Asset hygiene: clean URDF, primitive collisions, verified inertias, export MJCF (1–2 d)
2. Import → USD; GUI joint/drop sanity checks (0.5 d)
3. `isaaclab.sh --new` external project scaffold (0.5 d)
4. ArticulationCfg + DCMotor actuator configs from EU datasheets (1–2 d)
5. Clone G1 velocity task, flat terrain, minimal DR — get anything walking (1 d)
6. Reward/gain iteration → rough terrain + full DR + pushes (3–7 d)
7. Sim2sim MuJoCo gate (1–2 d)
8. Locomotion eval suite + report (1–2 d)
9. Manipulation track in Arena, parallel (1–2 wk)

GPU: one RTX 4090-class (≥16 GB VRAM) is fully sufficient.
