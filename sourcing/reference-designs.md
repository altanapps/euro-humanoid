# Reference design survey — open-source humanoids (June 2026)

Research record. Decision derived from this lives in `design/design-v0.md`.

## Comparison table

| Design | Year | Height / Mass | DoF | Actuation | Open assets | BOM cost | Walks? | Sim config | Manipulation |
|---|---|---|---|---|---|---|---|---|---|
| **Berkeley Humanoid Lite** | 2025 | 0.8 m / 16 kg | 22 (6/leg, 5/arm) | 3D-printed cycloidal + BLDC drone motors, ~20 Nm | CAD, URDF/MJCF/USD, firmware, priced BOM, Isaac Lab RL, teleop | **$4.3k** | Yes (RL, sim2real) | **Isaac Lab native** | Teleop bimanual |
| Berkeley Humanoid (orig.) | 2024 | 0.85 m / 16 kg | 12 (no arms) | Custom QDD | Code+URDF only, no CAD/BOM | n/a | Yes (robust) | Isaac Lab | None |
| Duke Humanoid v1 | 2024 | 1.0 m / 30 kg | 10 (no arms) | Motorevo planetary BLDC, 72–80 Nm | CAD wiki, Isaac Gym RL | n/a | Yes (0.3 m/s) | Isaac Gym (legacy) | None |
| **K-Scale K-Bot** | 2025 | 1.4 m / 34 kg | 20 (5/leg, 5/arm) | Off-the-shelf Robstride QDD (14–120 Nm peak) | Full CAD (CERN-OHL-S), URDF/MJCF, ksim (MuJoCo+JAX), RL policy | sold $8,999 | Yes (early polish) | MuJoCo/ksim; Isaac port easy | Simple grippers |
| Fourier N1 | 2025 | 1.3 m / 38 kg | 23 | Proprietary FSA 2.0, 96 Nm | Full CAD+BOM+URDF+Isaac Gym RL | published list | **Strong** (3.5 m/s, 1000 h) | Isaac Gym | No hands |
| ARTEMIS (RoMeLa) | 2025 | 1.42 m / 38.5 kg | 20 | Custom BEAR actuators | Full CAD, URDF, electronics | n/a (costly) | Yes (2.1 m/s, RoboCup champ) | None public | No hands |
| TienKung 3.0 | 2026 | ~1.65 m / 43 kg | 43 | Proprietary | URDF+STL, **TienKung-Lab Isaac Lab AMP** | n/a | Yes | **Isaac Lab native** | 5-finger (paid tier) |
| AGILOped (Uni Bonn) | 2025 | 1.1 m / 14.5 kg | 10 | MyActuator RMD X6-40 ×10 | CAD, ROS | ~$6.4k | Yes (model-based) | No Isaac | 1-DoF arms |
| ToddlerBot 2.0 | 2025 | 0.56 m / 3.4 kg | 30 | Dynamixel position servos | Everything | $6k | Yes (0.25 m/s) | MuJoCo/MJX | **Best-in-class** (90% bimanual) |
| Unitree G1 (benchmark, closed) | 2024 | 1.32 m / 35 kg | 23–43 | Integrated QDD, knee 90–120 Nm | URDF/MJCF + **official Isaac Lab RL** | $16k retail | Yes (2 m/s, mature) | Isaac Lab + MuJoCo, huge community | Dex3-1 (EDU) |

## Ranked recommendation (fork candidates)

1. **Berkeley Humanoid Lite** — only design hitting all three criteria natively: first-party Isaac Lab training + MJCF sim2sim + documented zero-shot sim2real; arms with demonstrated teleop manipulation; BOM is generic e-commerce parts → European substitution is a sourcing exercise, not engineering. Weaknesses: 0.8 m (small), printed cycloidal gears wear, modest gait. Licenses MIT + CC BY-SA.
2. **K-Scale K-Bot** — best full-size (1.4 m) open CAD; uses unmodified off-the-shelf QDD modules with published per-joint torque/ratio, so spec-matched European modules drop in with bracket changes only. Company dead (Nov 2025) → frozen repos, no support; MuJoCo/JAX stack, Isaac Lab port = days.
3. **Fourier N1** — hardware-proven far beyond the others but actuators single-sourced from Fourier (China); use as structural/spec reference only.

**Spec bar:** Unitree G1's torque ladder (~25 / 60 / 90–120 Nm tiers), 23 DoF, 1.32 m, 35 kg, 2 m/s.

## Key sources

- https://lite.berkeley-humanoid.org/ · https://github.com/HybridRobotics/Berkeley-Humanoid-Lite · arxiv.org/abs/2504.17249
- https://docs.kscale.dev/robots/k-bot/mechanical/ · https://github.com/kscalelabs/ksim · https://github.com/kscalelabs/kbot-joystick
- https://github.com/FFTAI (Fourier N1) · https://artemis.romela.org/ · https://github.com/Open-X-Humanoid/TienKung-Lab
- https://arxiv.org/abs/2509.09364 (AGILOped) · https://toddlerbot.github.io/
- https://github.com/unitreerobotics/unitree_rl_lab · https://www.unitree.com/g1/
