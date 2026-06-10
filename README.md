# euro-humanoid

A walking humanoid robot designed entirely from European-sourced parts (EU + UK + CH + NO + Turkey), validated in simulation (Isaac Sim / Isaac Lab) before any metal is cut.

## Goal

1. **Reference design** — fork a proven open-source humanoid morphology rather than inventing one.
2. **European co-design** — map every joint and subsystem to parts sourceable from European suppliers; let actuator availability drive the design, not the other way around.
3. **Simulation-first validation** — URDF/USD model with real actuator parameters, RL walking policy trained and evaluated in Isaac Lab, plus manipulation evals (object pick-up).
4. **Costed BOM** — full part list with suppliers, countries of origin, and prices.

## Approach

Sourcing leads design. The European actuator supply chain (torque density, gear options, drive electronics) sets the constraints; the morphology adapts to what is actually buyable. Sim runs in parallel from day one — it only needs actuator models and mass distribution, not a finished BOM.

## Repo layout (planned)

```
design/        # requirements, morphology, joint spec sheets
sourcing/      # European supplier research, per-category part options
bom/           # the costed bill of materials
sim/           # URDF/USD, Isaac Lab configs, training + eval code
evals/         # walking + manipulation eval definitions and results
```

## Status

- [x] Repo scaffold
- [x] Reference design survey → `sourcing/reference-designs.md`
- [x] European actuation supply-chain map → `sourcing/actuation-europe.md`
- [x] Sensors / compute / battery / structure sourcing → `sourcing/non-actuation-europe.md`
- [x] Design v0 (EUH-1: 23 DoF, 1.25 m, ~30 kg) → `design/design-v0.md`
- [x] BOM v0 (≈ €62k prototype, gripper config) → `bom/bom-v0.md`
- [x] Sim pipeline plan → `sim/pipeline.md`
- [ ] **Phase 0:** Berkeley Humanoid Lite Isaac Lab env + EUH-1 actuator models — does it still walk?
- [ ] Phase 1: own CAD → URDF/MJCF → Isaac Lab training + eval suite
- [ ] Phase 2: bench-test 1× class A + 1× class B joint, sysID, retrain
- [ ] Phase 3: full build

## Headline findings (2026-06-10)

- **Feasible.** Every subsystem has a credible European source except GPU inference — one declared exception (Jetson Orin NX, €800, 1.3% of BOM).
- **Cost:** ≈ €62k prototype / €35–40k at ~100 units. The European premium (~€24k vs Chinese actuators) is concentrated entirely in actuation — Europe has no cheap integrated QDD module ecosystem.
- **Architecture:** QDD hybrid buy/build — buy Synapticon ACTILINK-JD (DE) for medium joints, build high/small-torque joints from TQ-RoboDrive (DE) motors + Neugart (DE) planetaries + MAB Robotics (PL) drives + RLS (SI) encoders.
- **Fork base:** Berkeley Humanoid Lite (Isaac Lab pipeline) + K-Bot (full-size CAD), benchmarked against Unitree G1.
