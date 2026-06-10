# Component envelopes — packaging-critical dimensions (datasheet-verified, June 2026)

Drives morphology + CAD. Confidence: ✓ = datasheet-exact, ~ = estimated.

## Actuation

| Part | Envelope | Mass | Notes |
|---|---|---|---|
| TQ ILM-E85x30 (class A motor) | stator OD 85 × L 42.7 mm, rotor ID 52 (hollow) | 822 g ✓ | 3.3/10.6 Nm, frameless — needs our housing |
| TQ ILM-E85x13 | OD 85 × L 25.7 | 356 g ✓ | 1.37/4.43 Nm — distal-joint option |
| TQ ILM-E50x14 (class C motor) | OD 50 × L 22.25, rotor ID 30 | 135 g ✓ | 0.53/1.72 Nm |
| Neugart PLE 80, 1-stage (class A gear) | Ø80 × housing L 60 (total L 133.5 w/ shaft) | 2.1 kg ✓ | i=8: 50 Nm nom / 80 max / 190 e-stop. **Heavy + long — see risk note** |
| Neugart PLE 60, 1-stage | Ø60 × L 47 (total 106) | 0.9 kg ✓ | i=8: 18/29 Nm |
| Synapticon ACTILINK-JD 10 (class B) | **Ø106 × L 72.8–86.3 mm** | **1.05–1.26 kg** ✓ | bigger than early sourcing figure (Ø98/0.96 kg) — corrected. JD 8 (Ø78.5, 510 g) and JD 9 (Ø88, 680 g) exist for lighter class-B slots |
| MAB MD80 v3.0 drive | round Ø55 PCB | 15 g ✓ | mounts on motor back |
| RLS AksIM-2 (output encoder) | ring 2 mm thk, stack 7.8–9.2 mm, ODs 29–80 | 10–30 g ✓ | axial read |

## Sensing / compute / power

| Part | Envelope | Mass | Notes |
|---|---|---|---|
| Bota MiniONE Pro (ankles) | Ø48 × 26.2 mm | 60 g ✓ | ISO 9409-1-15.5 flange |
| SICK Visionary-B Two (head) | 162 × 96.6 × 79.3 mm | **1.5 kg** ✓ | ⚠ heavy for a head — candidate for replacement with IDS stereo pair or torso mount |
| Jetson Orin NX + Seeed A603 carrier | 87 × 52 × 26 mm | ~130 g ~ | + heatsink/fan ~25 mm, +90 g |
| Raspberry Pi CM5 | 55 × 40 × 4.7 | ~12 g ~ | |
| Battery 13S4P Aspilsan A28 (531 Wh) | ~265 × 95 × 85 mm (single-layer brick) | **~2.9 kg** ~ | 52 cells × 44.5 g + BMS/case; torso spine item |
| Schunk EGP 40 gripper | 40 × 26 × 88 mm | 320 g ✓ | 140 N grip |

## Design consequences (v0 → v1 corrections)

1. **PLE 80 kills the class-A mass budget**: 2.1 kg gear + 0.82 kg motor + housing ≈ 3.2 kg/joint × 6 = ~19 kg in class A alone. Options: (a) custom low-ratio planetary (lighter, the Berkeley/Unitree approach — PLE is an industrial catalog part with steel housing), (b) PLE 60 on knees with higher motor torque, (c) Gysin/custom 2-stage compact. **v1 assumes a custom 10:1 planetary at ~0.6 kg (machined by FACTUREE) — catalog PLE stays the zero-engineering fallback.**
2. JD-10 corrected envelope (Ø106) sets hip-yaw and shoulder pod diameters.
3. Head sensor: SICK 1.5 kg at head height wrecks the inverted pendulum — move vision to upper torso or swap to IDS stereo pair (~300 g). v1: torso-mounted.
4. ILM-E85x13 (356 g) is attractive for ankle-pitch if we accept 4.4 Nm × 10:1 ≈ 40 Nm peak — borderline; keep JD-10 at ankle pitch for now.
