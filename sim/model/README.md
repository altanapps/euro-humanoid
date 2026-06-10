# EUH-1 parametric body model

Our own morphology — not G1's, not Berkeley's — generated from `euh1_params.py` (single source of truth) by `generate_euh1.py`, which emits both `euh1.xml` (MJCF) and `euh1.urdf` from the same parameters.

- 23 actuated DoF, **30.58 kg**, 1.25 m, feet-flat stand keyframe
- Actuator masses placed at joints: class A 6×1.6 kg (ILM-E85 + custom planetary), class B 7×1.15 kg (Synapticon JD-10, corrected Ø106 envelope), class C 10×0.5 kg
- Torso carries battery (2.9 kg) + compute/vision (1.7 kg, torso-mounted per `design/envelopes.md`)
- Per-class torque caps baked into actuators: 80 / 48 / 20 Nm; velocity limits 13.5 / 20.4 / 26 rad/s

## Regenerate / validate / preview

```bash
sim/.venv/bin/python sim/model/generate_euh1.py     # emits euh1.xml + euh1.urdf
sim/.venv/bin/python sim/model/validate_euh1.py     # DoF/mass checks
sim/.venv/bin/python sim/model/render_preview.py    # euh1_preview.mp4 (stand + pushes)
```

## Stability findings (hard-won, 2026-06-10)

1. A passive PD stand at deployment gains (kp 120/60/30) is **statically unstable** on this morphology: knee gravity load (~39 Nm crouched) sags the P-controller, and ankle stiffness 2×60 < m·g·h ≈ 188 Nm/rad fails the inverted-pendulum criterion. No constant-target offset fixes it (verified: exact gravity feedforward still diverges).
2. Statue test (rigid joints, EU torque caps enforced) stands indefinitely — geometry, contacts, mass distribution are sound.
3. Demo stands use **5× gains** (kp 600/300/150): survives four 45 N pelvis pushes with max torque ~20 Nm — a quarter of the class-A cap. The European limits are never the binding constraint for standing.
4. Dynamic balance at deployment gains is the RL policy's job (next: retrain the Phase 0b task on this body via `euh1.urdf`).
