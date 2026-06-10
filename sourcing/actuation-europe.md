# European actuation supply chain (June 2026)

Scope: EU + UK + CH + NO + TR. ⚠ = non-European ownership caveat (located/made in Europe, foreign parent).

## Verdict

**A fully-European actuation stack is feasible today.** Every layer has at least one credible European-made option. Cost ≈ 3× Chinese QDD modules. Biggest structural gap: no European equivalent of the cheap mass-produced integrated QDD module (€300–600, 60–150 Nm class) — Synapticon ACTILINK is the only close analog at 2–4× the price. Turkey contributes essentially nothing merchant-grade in actuation (ASELSAN in-house only; EMF Motor too big/industrial).

## 1. Integrated joint actuators

| Product | Country | Specs | Price | 
|---|---|---|---|
| **Synapticon ACTILINK-JD 10** (QDD, humanoid-marketed) | 🇩🇪 | 20 Nm rated / **60 Nm peak**, 9:1, 0.96 kg, 48 V, EtherCAT, integrated Circulo drive + encoder, STO | **≈ €1,210** |
| **Synapticon ACTILINK-JP 17** (strain-wave) | 🇩🇪 | 22 Nm rated / **66 Nm peak**, 101:1, 1.56 kg, dual 19-bit encoders, brake | **≈ €2,080** |
| **maxon HEJ 90** | 🇨🇭 | **140 Nm peak**, 1.96 kg, EtherCAT, IP67, impedance control | est. €3,000–4,500 |
| maxon HEJ 70-48-50 | 🇨🇭 | 50 Nm peak, ~1.05 kg | est. €2,000–3,000 |
| maxon integrated joint gearbox (late 2025) | 🇨🇭 | **270 Nm peak**, Ø90×40 mm — hip/knee class | est. €2,500–4,000 |
| Sensodrive SensoJoint 5000/6000 (DLR spin-off, output torque sensor) | 🇩🇪 | 62–176 Nm max, 1.8–3.4 kg, HD gear, EtherCAT | est. €5,000–9,000 |
| igus ReBeL polymer strain-wave joint | 🇩🇪 | ~15–25 Nm, very light, CAN | €713–1,027 |

## 2. Frameless torque motors

| Product | Country | Notes | Price |
|---|---|---|---|
| **TQ-RoboDrive ILM-E** 50/70/85 | 🇩🇪 | DLR heritage (KUKA iiwa); cost-down volume line; e.g. ILM-E85x30: 444 W, 3.3 Nm cont., hollow shaft | est. €500–1,800 |
| **maxon EC frameless flat** 45/60/90 | 🇨🇭 | EC90 flat: ~0.5–1 Nm cont. (~3× peak) | est. €300–550 |
| Alva Industries SlimTorq | 🇳🇴 | FiberPrinting stators, humanoid-marketed (Robotics Summit 2026), RLS partnership | est. €600–1,500 |
| Wittenstein cyber kit | 🇩🇪 | 16–175 Nm max across line | est. €800–2,000 |
| Phase Motion Control TK | 🇮🇹 | custom thin-ring torque motors | est. €700–2,000 |
| Faulhaber BXT | 🇩🇪 | wrist/finger class only | est. €250–400 |

## 3. Gearing

| Product | Country | Notes | Price |
|---|---|---|---|
| Harmonic Drive SE CSG/CPL/CSD | 🇩🇪 ⚠ (JP parent; genuine German production, Limburg) | 23–3,419 Nm peak across sizes | OEM est. €400–900/set (sz 17–25) |
| **Spinea TwinSpin/DriveSpin** | 🇸🇰 ⚠ (Timken/US since 2022; Slovak production) | only European precision cycloidal maker; TS050 36 Nm peak | est. €900–2,000 |
| **IMSystems Archimedes Drive** | 🇳🇱 | traction-roller reducer, zero backlash, explicitly humanoid-marketed; pre-volume | est. €500–1,500 |
| **Neugart PLE/PLF economy planetary** | 🇩🇪 | to 260 Nm, 3–512:1 — the budget QDD stage | **€150–600** |
| Wittenstein alpha RP+/LP+ | 🇩🇪 | precision, hollow shaft | est. €800–2,500 |
| Gysin GPL | 🇨🇭 | 0.1–350 Nm, custom low-ratio possible | est. €600–1,500 |

## 4. Servo drives (48 V torque control)

| Product | Country | Notes | Price |
|---|---|---|---|
| **MAB Robotics MD80** | 🇵🇱 | legged-robot FOC controller, CAN-FD, impedance control, ROS2 SDK | **€275** |
| Synapticon SOMANET Circulo 7/9 | 🇩🇪 | ring-shaped hollow-shaft, dual encoders, SIL3 FSoE, EtherCAT | est. €600–1,000 |
| Novanta/Ingenia Everest XCR | 🇪🇸 ⚠ (US parent; Barcelona design+mfg) | 30 A cont./60 A peak, 8–72 V, EtherCAT | est. €700–1,100 |
| SOLO UNO v2 | 🇳🇱 | 800/1200 W, FOC, CANopen | **from €209** |
| ST B-G431B-ESC1 / STSPIN32G4 | 🇫🇷🇮🇹 | hobby-grade FOC (Berkeley Lite uses these) | **€18–50** |
| Trinamic TMC4671 | 🇩🇪 ⚠ (ADI/US) | servo-on-chip | €15–25 |

## 5. Absolute encoders

| Product | Country | Notes | Price |
|---|---|---|---|
| **RLS AksIM-2** off-axis ring | 🇸🇮 ⚠ (Renishaw/UK — still European) | 17–20 bit, low profile, "9/10 cobots" | **€221** |
| RLS AksIM-4 dual-concentric | 🇸🇮 | motor+output in one — purpose-built for robot joints | est. €300–450 |
| Heidenhain KCI 120 Dplus | 🇩🇪 | bearingless inductive dual encoder, SIL3 | est. €250–450 |
| POSITAL-Fraba kit encoders | 🇩🇪 (mfg 🇵🇱) | battery-free Wiegand multiturn, humanoid-marketed | est. €80–150 |
| ams-OSRAM AS5047P | 🇦🇹 | standard cheap QDD motor-side chip | **€10–15** |

## European humanoid-actuator players to watch

Synapticon (DE), maxon HEJ (CH), Sensodrive (DE), Duatic DuaDrive (CH, ETH spin-off), MAB Robotics (PL), IMSystems (NL), Alva (NO), TQ ILM-E (DE). Demand side (in-house only): 1X (NO), PAL Robotics (ES), Neura (DE), Akın Robotics (TR).

## Per-joint cost (prototype singles, EUR)

**QDD architecture (recommended):** frameless motor + 6–10:1 planetary + drive + dual encoders + housing

| Class | Build | Singles | ~100-unit |
|---|---|---|---|
| Hip/knee 100–150 Nm peak | ILM-E85/EC90 + Neugart + MD80 + AS5047P + AksIM-2 | **€1,500–3,000** | €900–1,600 |
| Medium ~50–60 Nm | buy **Synapticon JD-10 @ €1,210** (beats DIY) | €1,210 | €700–1,100 |
| Small ~20 Nm | small frameless + planetary + drive + encoders | **€800–1,300** | €450–800 |

Harmonic-based alternative ≈ 1.6× these numbers.

## 23-DoF totals

| Architecture | Prototype | ~100-unit volume |
|---|---|---|
| Fully-EU QDD | **€26k–37k** | €17k–22k |
| Harmonic-based | €58k | €30k–38k |
| (Reference: Chinese QDD modules) | €8k–14k | €5k–9k |

Key URLs: synapticon.com/en/applications/humanoids · tq-group.com/en/products/tq-robodrive/ · mabrobotics.pl/md-series · rls.si · imsystems.nl/industries/humanoids-robots/ · maxongroup.com · sensodrive.de · spinea.com · neugart.com · alvaindustries.com
