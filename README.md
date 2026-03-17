# Indoor GPS-Denied Nano-Quadcopter Mapping

## Overview

This project investigates whether a nano-scale quadcopter can perform autonomous indoor localization and mapping without GPS under strict size, power, and computational constraints.

The system combines optical flow velocity estimation, time-of-flight (ToF) depth sensing, and a lightweight companion-computer pipeline to reconstruct a 2D map of the environment from flight data.

The goal is to enable small UAVs to operate in GPS-denied environments for applications such as search-and-rescue (SAR), structural inspection, and confined-space exploration.

---

## Features

- GPS-denied indoor navigation  
- Optical flow-based velocity estimation  
- Multi-sensor ToF depth acquisition (VL53L5CX array)  
- Modular sensor + compute architecture  
- Companion-computer data logging pipeline  
- Post-flight 2D mapping reconstruction  

---

## System Architecture

The system consists of three main subsystems:

1. Flight Controller  
   Stabilization and low-level control (ArduPilot-based)

2. Sensor Hub (ESP32-S3)  
   Aggregates ToF and auxiliary sensor data

3. Companion Computer (LicheeRV Nano)  
   Handles logging and mapping pipeline

Data flow:  
Sensors → ESP32-S3 → Companion Computer → Post-processing → 2D Map

---

## Hardware

### Version 1
| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping SBC | Luckfox Pico Mini B |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1202.5 11500KV |
| Frame | 85mm Mobula8 Whoop |

---

### Version 2
| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping SBC | LicheeRV Nano |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1103 11000KV |
| Frame | 2" Carbon Fiber |

---

### Version 3 (Current)
| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping SBC | LicheeRV Nano |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1202.5 11500KV |
| Frame | 2.5" Carbon Fiber |
| Propellers | 2.5" Bi-blade |

Approximate AUW: ~135g  
Battery: 2S LiHV  

---

## Control & Tuning

### January 30, 2026

Roll & Pitch

| Parameter | Value |
|-----------|-------|
| ATC_RAT_RLL_P | 0.07 |
| ATC_RAT_PIT_P | 0.07 |
| ATC_RAT_RLL_I | 0.06 |
| ATC_RAT_PIT_I | 0.06 |
| ATC_RAT_RLL_D | 0.0045 |
| ATC_RAT_PIT_D | 0.0045 |

Yaw

| Parameter | Value |
|-----------|-------|
| ATC_RAT_YAW_P | 0.05 |
| ATC_RAT_YAW_I | 0.015 |
| ATC_RAT_YAW_D | 0.0 |

---

## Results

Test flights were conducted in small indoor environments.

Key observations:

- Stable hover achieved using optical flow velocity estimation  
- Short-duration mapping is feasible (~60-120 seconds)  
- Accumulated drift remains the primary limitation  
- ToF frame distortion introduces curvature artifacts in mapping  

---

## Mapped Environment

![Autonomously mapped environment](image%20(9).png)

---

## Build

### Compile Command

riscv64-linux-gnu-gcc -Os -std=gnu11 -w -static \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -I"$HOME/c_library_v2" \
  clean_uav_fc_tof_nav.c -o uav_fc_tof_nav_riscv_static -lm

---

## Repository Structure

src/  
    uav_fc_tof_nav.c  
    sensor interface code  

logs/  
    flight logs  
    sensor captures  

tools/  
    visualization / mapping scripts  

---

## Limitations

- Drift accumulation over time (no global correction)  
- Limited compute prevents real-time SLAM  
- Sensor FOV distortion affects map accuracy  
- Short flight duration due to size constraints  

---

## Future Work

- Scan-to-scan matching (ICP / graph-based SLAM)  
- Sensor fusion improvements (full 3D ray projection)  
- Drift correction using loop closure  
- Real-time onboard mapping  
- Weight reduction and efficiency optimization  

---

## Citation

If referencing this project:

Ethan Xie. Indoor GPS-Denied Nano-Quadcopter Mapping. 2026.
