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
- [ ] Reference design survey
- [ ] European actuation supply-chain map
- [ ] Sensors / compute / battery / structure sourcing
- [ ] Joint spec sheet + morphology freeze
- [ ] URDF/USD + Isaac Lab walking policy
- [ ] Manipulation evals
- [ ] Final costed BOM
