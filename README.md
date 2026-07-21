# Indoor GPS-Denied Nano-Quadcopter Mapping

## Overview

This project investigates whether a nano-scale quadcopter can perform autonomous indoor localization and mapping without GPS under strict size, power, and computational constraints.

The system combines optical flow velocity estimation, time-of-flight (ToF) depth sensing, and a lightweight companion-computer logging pipeline to reconstruct a 2D map of the environment from flight data.

The goal is to explore whether a highly constrained nano-quadcopter platform can support GPS-denied navigation and mapping for applications such as search-and-rescue, structural inspection, and confined-space exploration.

![Drone (older version)](assets/drone%20photo.png)
*Figure: Earlier prototype of the nano-quadcopter platform; hardware has since been updated.*

---

## Features

- GPS-denied indoor flight
- Optical flow-based velocity estimation
- ToF-based environment sensing
- Modular sensor and compute architecture
- Companion-computer logging pipeline
- Post-flight 2D map reconstruction

---

## System Architecture

The system consists of three main subsystems:

1. **Flight Controller**  
   Handles stabilization and low-level control

2. **Sensor Hub (ESP32-S3)**  
   Collects and transmits ToF and related sensor data

3. **Companion Computer (LicheeRV Nano / earlier Luckfox Pico Mini B)**  
   Logs flight and sensor data for post-flight reconstruction and analysis

Data flow:

Sensors → ESP32-S3 → Companion Computer → Post-processing → 2D Map

---

## Hardware

### Version 1

| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping / Vision SBC | Luckfox Pico Mini B |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1202.5 11500KV |
| Frame | 85mm Mobula8 Whoop Frame |

### Version 2

| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping / Vision SBC | LicheeRV Nano |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1103 11000KV |
| Frame | 2" Carbon Fiber |

### Version 3 (Current)

| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping / Vision SBC | LicheeRV Nano |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1202.5 11500KV |
| Frame | 2.5" Carbon Fiber |
| Propellers | 2.5" Bi-blade |

Approximate AUW: ~135g  
Battery: 2S LiHV

---

## Control & Tuning

### January 30, 2026

### Roll & Pitch

| Parameter | Value |
|-----------|-------|
| ATC_RAT_RLL_P | 0.07 |
| ATC_RAT_PIT_P | 0.07 |
| ATC_RAT_RLL_I | 0.06 |
| ATC_RAT_PIT_I | 0.06 |
| ATC_RAT_RLL_D | 0.0045 |
| ATC_RAT_PIT_D | 0.0045 |

### Yaw

| Parameter | Value |
|-----------|-------|
| ATC_RAT_YAW_P | 0.05 |
| ATC_RAT_YAW_I | 0.015 |
| ATC_RAT_YAW_D | 0.0 |

---

## Results

Test flights were conducted in small indoor environments.

Key observations:

- Stable hover was achieved using optical flow-based velocity estimation
- Short-duration mapping is feasible on a nano-scale platform
- Drift accumulation remains a major limitation
- ToF sensing introduces distortion and curvature artifacts in reconstructed maps
- The project demonstrates the practicality of modular indoor mapping on a constrained quadcopter platform

---

## Mapped Environment

![Autonomously mapped environment](assets/Figure_1.png)
![Autonomously mapped environment](assets/image%20(9).png)

---

## Repository Structure

- `c/` — flight control and navigation code
- `python/` — scan parsing and visualization tools
- `arduino/` — ESP32 / helper firmware
- `assets/` — images and documentation figures

Key files:

- `c/autonomy/` — authoritative, bounded C autonomy core and guarded runner
- `c/clean_uav_fc_tof_nav.c` — legacy real-flight hover code (reference only)
- `c/manual_uav_fc_tof_nav.c` — legacy manual/logging code
- `c/uav_local_nav.c` — legacy local-navigation code
- `c/frontier.c` — legacy real-flight frontier code; explicit unsafe gate required
- `python/plot_scan.py` — 2D mapping / scan visualization script
- `python/plot_scan_3d.py` — 3D point-cloud viewer and room-alignment tool
- `arduino/tof_esp32.ino` — ESP32 sensor hub firmware
- `arduino/m5stack_armDisarm.ino` — M5Stack arm/disarm helper
- `assets/image (9).png` — example mapped environment output

---

## Safe build and execution

The supported launcher defaults to a fake backend with exploration disabled. It
does not open UARTs:

```bash
make test
./scripts/run_autonomy.sh
./scripts/run_autonomy.sh --explore              # synthetic dry-run only
./scripts/replay_autonomy.sh log/3_frontierTest_scanlog.bin
make riscv
```

MAVLink and SITL modes currently fail closed because the hardened nonblocking
ACK/state backend and production mapping adapter are not integrated. Passing live
opt-in flags does not bypass that blocker. Do not use the legacy binaries for a
flight. See `docs/live_hardware_checklist.md` and `AUTONOMY_INTEGRATION_REPORT.md`.
