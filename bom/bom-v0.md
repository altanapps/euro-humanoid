# EUH-1 BOM v0 — costed (prototype, qty 1, June 2026 prices)

Prices: found where possible, estimated (est.) where quote-only. Sources in `sourcing/*.md`.

## Actuation — €31,640

| Item | Qty | Unit € | Total € | Supplier (country) |
|---|---|---|---|---|
| Class A custom QDD ~100 Nm (ILM-E85 + Neugart PLE + MD80 + AS5047P + AksIM-2 + housing) | 6 | 2,200 est. | 13,200 | TQ 🇩🇪 / Neugart 🇩🇪 / MAB 🇵🇱 / ams 🇦🇹 / RLS 🇸🇮 |
| Class B Synapticon ACTILINK-JD 10 (60 Nm peak) | 7 | 1,210 | 8,470 | Synapticon 🇩🇪 |
| Class C custom QDD ~20 Nm | 10 | 1,000 est. | 10,000 | maxon 🇨🇭 / TQ 🇩🇪 / MAB 🇵🇱 |

## Sensing — €10,400

| Item | Qty | Unit € | Total € | Supplier |
|---|---|---|---|---|
| Xsens MTi-630 AHRS (torso) | 1 | 1,000 | 1,000 | Movella 🇳🇱 ⚠US-PE |
| Murata SCHA63T limb IMU | 4 | 55 | 220 | Murata 🇫🇮 ⚠JP |
| Bota MiniONE Pro 6-axis F/T (ankles) | 2 | 2,500 est. | 5,000 | Bota 🇨🇭 |
| ME-Meßsysteme sole load cells | 8 | 100 | 800 | ME 🇩🇪 |
| SICK Visionary-B Two RGB-D | 1 | 3,000 est. | 3,000 | SICK 🇩🇪 |
| IDS uEye RGB head cam | 1 | 350 | 350 | IDS 🇩🇪 |

## Compute — €1,200

| Item | Qty | Unit € | Total € | Supplier |
|---|---|---|---|---|
| STM32H755 RT board | 1 | 50 | 50 | ST 🇫🇷🇮🇹 |
| Raspberry Pi CM5 8GB + carrier | 1 | 180 | 180 | RPi 🇬🇧 |
| **NVIDIA Jetson Orin NX 16GB — declared exception** | 1 | 800 | 800 | NVIDIA 🇺🇸 |
| Axelera Metis M.2 (optional vision offload) | 1 | 250 | 250 | Axelera 🇳🇱 |

## Power — €2,700

| Item | Qty | Total € | Supplier |
|---|---|---|---|
| BMZ custom 13S 48 V ~530 Wh pack (Aspilsan 🇹🇷 cells) + CAN BMS | 1 | 2,300 est. | BMZ 🇩🇪 |
| Traco/RECOM DC-DC set (48→24/12/5) | 1 | 400 | Traco 🇨🇭 / RECOM 🇦🇹 |

## End effectors — €2,600 (gripper config)

| Item | Qty | Total € | Supplier |
|---|---|---|---|
| Schunk EGP-class parallel gripper | 2 | 2,600 est. | Schunk 🇩🇪 |
| *(Upgrade path: 2× Seed Robotics RH8D dexterous, +€8,600)* | | *(11,200)* | *Seed 🇵🇹* |

## Structure + integration — €13,500

| Item | Total € | Supplier |
|---|---|---|
| CNC machined set (~60–100 parts, 7075/6061) | 9,000 est. | FACTUREE 🇩🇪 / Weerg 🇮🇹 / TR shops |
| Thin-section + general bearings | 1,500 | Franke 🇩🇪 / Schaeffler 🇩🇪 / SKF 🇸🇪 |
| Carbon fiber tube | 500 | CG TEC 🇩🇪 |
| 3D printing (Prusa, amortized) | 1,000 | Prusa 🇨🇿 |
| Cabling/connectors (LEMO, ODU, Harting, igus) | 1,500 | 🇨🇭🇩🇪 |

## Middleware — €150

EtherCAT: SOEM master (free, Beckhoff 🇩🇪 IP royalty-free) + ESC silicon across nodes.

---

## Totals

| Config | Prototype (qty 1) | Est. @ ~100 units |
|---|---|---|
| **EUH-1 gripper config** | **≈ €62,200** | ≈ €35–40k |
| EUH-1 dexterous (RH8D hands) | ≈ €70,800 | ≈ €43–48k |
| Reference: Unitree G1 retail | €16k | — |
| Reference: same robot with Chinese QDD modules | ≈ €38k | — |

**The European premium is ~€24k/unit at prototype scale, concentrated almost entirely in actuation** (no European cheap integrated QDD module exists — see `sourcing/actuation-europe.md` verdict).

European content: 100% of categories except the Jetson Orin NX (1 declared exception, €800 ≈ 1.3% of BOM value). Ownership-clean European content (excluding ⚠ foreign-owned-but-Europe-made): see exceptions list in `sourcing/non-actuation-europe.md`.
