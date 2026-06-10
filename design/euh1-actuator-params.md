# EUH-1 actuator parameters for simulation (v0, 2026-06-10)

Derived from EU part datasheets in `sourcing/actuation-europe.md`. Values marked **(est.)** need datasheet/quote confirmation — treat as ±30% until Phase 2 bench sysID.

## Class A — hip pitch ×2, hip roll ×2, knee ×2

TQ-RoboDrive ILM-E85x30 + Neugart PLE 10:1 planetary (η ≈ 0.95)

| Param | Value | Derivation |
|---|---|---|
| Continuous torque | **31 Nm** | 3.3 Nm × 10 × 0.95 |
| Peak torque (`saturation_effort`) | **100 Nm** (est.) | ~3× continuous, motor-limited |
| `effort_limit` (sim cap) | 80 Nm | conservative vs peak, thermal headroom |
| `velocity_limit` | **13.5 rad/s** | 1286 rpm @ 48 V ÷ 10 |
| `armature` (output side) | **2.3e-2 kg·m²** (est.) | J_rotor ≈ 2.3e-4 kg·m² × 10² |
| Unit mass (motor+gear+drive+housing) | ~1.4 kg | |
| Start gains | kp 120, kd 4.0 | G1-config neighborhood, tune |

## Class B — hip yaw ×2, ankle pitch ×2, waist yaw, shoulder pitch ×2

Synapticon ACTILINK-JD 10 (integrated QDD, 9:1)

| Param | Value | Derivation |
|---|---|---|
| Continuous torque | **20 Nm** | datasheet rated |
| Peak (`saturation_effort`) | **60 Nm** | datasheet peak |
| `effort_limit` | 48 Nm | 0.8 × peak |
| `velocity_limit` | **20.4 rad/s** | 195 rpm datasheet |
| `armature` | **6.5e-3 kg·m²** (est.) | J_rotor ≈ 8e-5 × 9² |
| Unit mass | 0.96 kg | datasheet |
| Start gains | kp 60, kd 2.5 | |

## Class C — ankle roll ×2, shoulder roll ×2, shoulder yaw ×2, elbow ×2, wrist roll ×2

ILM-E50 (or maxon EC60 flat) + 8:1 planetary

| Param | Value | Derivation |
|---|---|---|
| Continuous torque | **8 Nm** (est.) | ~1 Nm × 8 |
| Peak (`saturation_effort`) | **25 Nm** (est.) | ~3× cont |
| `effort_limit` | 20 Nm | |
| `velocity_limit` | **26 rad/s** (est.) | ~2000 rpm ÷ 8 |
| `armature` | **1.3e-3 kg·m²** (est.) | 2e-5 × 8² |
| Unit mass | ~0.6 kg | |
| Start gains | kp 30, kd 1.5 | |

## Comparison vs Unitree G1 (the morphology we test on)

| Joint | G1 peak | EUH-1 peak | Δ |
|---|---|---|---|
| Knee | 139 Nm | 100 Nm | **−28%** ← the experiment |
| Hip pitch/roll | 88 Nm | 100 Nm | +14% |
| Ankle pitch | 50 Nm | 60 Nm | +20% |
| Ankle roll | 50 Nm | 25 Nm | **−50%** ← watch this |
| Arms | ~25 Nm | 25–60 Nm | ≥ parity |

The Phase 0b question in one line: **does a 30 kg G1-class morphology walk with 100 Nm knees and 25 Nm ankle-roll, and how close to saturation does it run?**
