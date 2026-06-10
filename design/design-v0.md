# EUH-1 — design v0 (paper design, 2026-06-10)

## Concept

A 23-DoF, ~1.25 m, ~30 kg walking humanoid, every part sourced in Europe (EU+UK+CH+NO+TR), with **one declared exception: the NVIDIA Jetson inference computer**. Morphology forked from proven open designs (Berkeley Humanoid Lite pipeline + K-Bot/G1 scale), not invented. Validated entirely in Isaac Lab before any metal is cut.

**The honest differentiation claim:** EUH-1 will not out-perform a Unitree G1 mechanically. Its edge is being the only open humanoid with a fully-European, fully-documented supply chain — sovereignty is the feature (EU funding, defense-adjacent, procurement-restricted buyers). Plus one technical deviation: torque-transparent joints (dual encoders on every joint, 6-axis F/T at both ankles) making it a better *sim2real research platform* than price-equivalent closed robots.

## Spec targets (benchmark: Unitree G1)

| | EUH-1 target | G1 |
|---|---|---|
| Height | 1.25 m | 1.32 m |
| Mass | ~30 kg | ~35 kg |
| DoF | 23 | 23 |
| Walk speed | ≥1.0 m/s (RL policy) | 2.0 m/s |
| Knee/hip peak torque | ~100 Nm | 90–120 Nm |
| Runtime | ≥1 h (530 Wh) | ~2 h |
| Bus | 48 V, EtherCAT + CAN-FD | proprietary |

## Joint map (23 DoF)

| Class | Joints | Count | Peak torque | Actuator |
|---|---|---|---|---|
| **A — high** | hip pitch ×2, hip roll ×2, knee ×2 | 6 | ~100 Nm | Custom QDD: TQ ILM-E85 🇩🇪 + Neugart PLE 10:1 🇩🇪 + MAB MD80 🇵🇱 + AS5047P 🇦🇹 + RLS AksIM-2 🇸🇮 |
| **B — medium** | hip yaw ×2, ankle pitch ×2, waist yaw, shoulder pitch ×2 | 7 | 60 Nm | **Synapticon ACTILINK-JD 10** 🇩🇪 off the shelf |
| **C — small** | ankle roll ×2, shoulder roll ×2, shoulder yaw ×2, elbow ×2, wrist roll ×2 | 10 | ~20–25 Nm | Custom small QDD: ILM-E50/maxon EC60 + planetary + MD80 + encoders |

End effectors: 2× parallel grippers (Schunk EGP class) in v0; Seed RH8D dexterous hands as the upgrade path.

## Architecture decisions (and why)

1. **QDD over harmonic** — backdrivable, impact-tolerant for RL locomotion, ~1.6× cheaper, and Europe's planetary supply (Neugart/Gysin) is strong where its cheap strain-wave supply is weak.
2. **Hybrid buy/build** — buy class B (Synapticon JD-10 at €1,210 beats any DIY at that torque), build class A and C (nothing European exists at the right price/torque). This concentrates engineering effort on exactly 2 custom joint designs.
3. **48 V single bus**, EtherCAT backbone for class A/B (Beckhoff IP, royalty-free master, SOEM), CAN-FD acceptable on class C.
4. **Sensing: dual encoder per joint + 2× Bota MiniONE Pro at ankles** + Xsens MTi-630 torso AHRS + Murata SCHA63T limb IMUs — the torque-transparency deviation.
5. **Compute split:** STM32H755 (1 kHz joint loop) → RPi CM5 (state estimation, EtherCAT master) → Jetson Orin NX 16GB (policy + perception; the exception) + optional Axelera Metis for vision.
6. **Structure:** CNC 7075/6061 (FACTUREE/Weerg; Turkish shops for cost), carbon tube limbs (CG TEC), Franke thin-section bearings at joints.

## Mass budget (rough)

| Subsystem | kg |
|---|---|
| Class A actuators (6 × ~1.4) | 8.4 |
| Class B (7 × 0.96) | 6.7 |
| Class C (10 × ~0.6) | 6.0 |
| Structure + bearings | 5.0 |
| Battery pack (530 Wh) | 3.0 |
| Compute + sensors + cabling | 1.5 |
| Grippers (2×) | 1.0 |
| **Total** | **~31.6** |

## Open risks (ranked)

1. **Class A torque margin** — ILM-E85 + 10:1 gives ~100 Nm peak only at ~3× continuous rating; thermal headroom for stairs/push-recovery unproven. Fallback: maxon 270 Nm integrated joint gearbox (€2.5–4k) on knees only. **Resolve in sim first: train with 80 Nm caps and measure how often the policy saturates.**
2. Actuator mass pushes total above 30 kg → torque requirement creeps up (the classic spiral). Sim iteration loop catches this.
3. Synapticon lead times / single-source risk on class B.
4. Battery: BMZ+Aspilsan pack is the sovereign story but Aspilsan 18650s are energy cells; power-cell discharge under walking transients needs validation. Fallback: Asian cells in BMZ pack (disclose).
5. Printed/cheap planetary wear at class C (Berkeley Lite's known weakness) — budget metal gears.

## Phase plan

- **Phase 0 (now, no CAD needed):** fork Berkeley Humanoid Lite's Isaac Lab env, swap in EUH-1 actuator models (DCMotorCfg from the EU datasheets above) + mass-scale to 30 kg → does it still walk? This validates the joint spec before any CAD exists.
- **Phase 1:** Onshape CAD (fork Lite/K-Bot geometry, rescale) → onshape-to-robot → URDF+MJCF → own Isaac Lab env → full training + eval per `sim/pipeline.md`.
- **Phase 2:** order 1× class A and 1× class B joint, bench-test, sysID → `IdentifiedActuatorCfg` → retrain.
- **Phase 3:** full build (BOM `bom/bom-v0.md`).
