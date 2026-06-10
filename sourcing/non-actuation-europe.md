# European non-actuation supply chain (June 2026)

Scope: EU + UK + CH + NO + TR. ⚠ = foreign-ownership caveat.

## Recommended picks (prototype qty)

| # | Category | Pick | Cost |
|---|---|---|---|
| 1 | IMU | Xsens MTi-630 torso AHRS (~€1,000) ⚠ US-PE-owned + Murata SCHA63T 🇫🇮 (€55) limb IMUs — cleanest EU-owned alt: SBG Ellipse Micro 🇫🇷 (~€2,200) | €1,200 |
| 2 | Vision | SICK Visionary-B Two 🇩🇪 (RGB+depth, rugged, ~€3,000) + IDS uEye RGB 🇩🇪 (~€350) | €3,400 |
| 3 | F/T + contact | 2× Bota Systems MiniONE Pro 🇨🇭 (ankles, ~€2,500 ea.) + ME-Meßsysteme KM26z load cells 🇩🇪 (soles) | €5,800 |
| 4 | Compute | STM32H755 🇫🇷🇮🇹 (1 kHz loop, €15) + Raspberry Pi CM5 8GB 🇬🇧 (Sony Wales, €105) + **NVIDIA Jetson Orin NX 16GB — THE exception** (€800) + optional Axelera Metis M.2 🇳🇱 (€250) | €1,200 |
| 5 | Battery + power | BMZ Group 🇩🇪 custom 13S 48 V ~530 Wh pack w/ **Aspilsan INR18650A28 🇹🇷 cells** (only made-in-Europe merchant cell, €5–9/cell) + Traco 🇨🇭 / RECOM 🇦🇹 DC-DC | €2,700 |
| 6 | Hands | 2× Seed Robotics RH8D 🇵🇹 (19 DoF dexterous, 620 g, €5,599 ea.) — budget: 2× Schunk EGP-class 🇩🇪 parallel ≈ €2,600 | €11,200 / €2,600 |
| 7 | Structure | FACTUREE 🇩🇪 / Weerg 🇮🇹 CNC (€8–15k set; Turkish job shops 30–50% cheaper) + Franke/Schaeffler 🇩🇪 + SKF 🇸🇪 bearings + CG TEC 🇩🇪 carbon tube + Prusa 🇨🇿 printing | €12,000 |
| 8 | Cabling | LEMO 🇨🇭 + ODU/Harting 🇩🇪 + igus 🇩🇪 — 100% European, zero compromise | €1,500 |
| 9 | Middleware | EtherCAT (Beckhoff 🇩🇪 IP, master royalty-free, SOEM free) + Hilscher netX/Beckhoff ET1100 ESC silicon | €150 |

**Total non-actuation: ≈ €30.6k (parallel grippers) / €39.2k (dexterous hands).** Honest range €28k–45k.

## Battery landscape notes (volatile)

- Northvolt: bankrupt Mar 2025, assets → Lyten (US), not procurable.
- Customcells: insolvent Apr 2025, restructured Dec 2025, niche/defense only.
- Varta V4Drive → Porsche "V4Smart" Mar 2025, automotive-captive.
- **Aspilsan (Kayseri, TR)**: only merchant made-in-Europe-territory 18650 (2,800 mAh NMC, EU-accredited). Practical default remains Asian cells in a European (BMZ) pack — flag in any sovereignty pitch.

## Exceptions list — no credible European option

1. **GPU/NPU policy+perception inference** — Jetson Orin NX unavoidable. Axelera is EU-designed but TSMC-fabbed and vision-CNN-only; Kalray exited edge; SiPearl is HPC; Hailo is Israeli.
2. **Advanced-node fab** — all "European" silicon (Axelera, RPi, much ST/Bosch back-end) is fabbed/packaged in Asia.
3. **Cheap RGB-D module** (€300–500 RealSense class) — European depth exists only at industrial prices (€2.5k–6k).
4. **High-power merchant 21700 cells at volume.**
5. **COTS high-res tactile fingertips** (GelSight class) — only early-stage (Touchlab UK, IEE LU).
6. **Europe-located but foreign-owned** (usable, disclose): Xsens/Movella (US PE), Photoneo (Zebra/US), Xometry/Protolabs EU (US), Murata Finland (JP), HD SE (JP), Spinea (Timken/US), Ingenia (Novanta/US).

Key URLs: sbg-systems.com · murata.com (SCHA63T) · baslerweb.com/stereo-ace · sick.com · botasys.com · me-systeme.de · axelera.ai · raspberrypi.com/products/compute-module-5 · bmz-group.com · aspilsan.com · seedrobotics.com · schunk.com · facturee.de · weerg.com · franke-gmbh.com · cg-tec.de · lemo.com · acontis.com
