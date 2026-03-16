# Indoor GPS-Denied Nano-Quadcopter Mapping

## Abstract

The use of unmanned aerial vehicles (UAVs) is rapidly expanding across search-and-rescue (SAR), defence, and surveillance applications. In these domains, autonomous operations in confined spaces are increasingly required for tasks such as victim detection, structural inspection, and situational awareness for rescuers. However, operation in indoor environments removes access to GPS, creating significant challenges for accurate localization and mapping, especially on small aerial platforms with limited onboard memory and sensing. This project investigates whether a modular nano-quadcopter can perform indoor localization and mapping without GPS under strict size, computational, and energy constraints.

A fully custom autonomous system was developed, integrating optical flow-based velocity estimates, short-range time-of-flight depth ranging, and an onboard Single Board Computer (SBC)-based data-logging pipeline. The system architecture was designed with modularity in mind, allowing sensors or modules to be expanded and reconfigured without the need for disassembly.

Flight and sensor data were recorded and processed to create a post-exploration two-dimensional map of the environment. System performance was repeatedly evaluated through test flights, iterative refinement, and documentation.

This system demonstrates that short-duration indoor mapping on a nano-scale aerial platform is feasible, while highlighting the challenges of drift and sensor limitations as long-term obstacles for indoor navigation.

---

## Hardware

### Version 1

| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping / Vision SBC | Luckfox Pico Mini B |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1202.5 11500kv |
| Frame | 85mm Mobula 8 Whoop Frame |

### Version 2

| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping / Vision SBC | LicheeRV Nano |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1103 11000kv |
| Frame | 2" Carbon Fiber |

### Version 3

| Component | Part |
|-----------|------|
| Sensor Hub | Waveshare ESP32-S3 Zero |
| Mapping / Vision SBC | LicheeRV Nano |
| Flight Controller | MicoAir H743 45A V2 AIO |
| Motors | 1202.5 11500kv |
| Frame | 2.5" Carbon Fiber |
| Propellers | 2.5" Bi-blade |

---

## Tuning Log

### January 30, 2026

**Roll & Pitch**

| Parameter | Value |
|-----------|-------|
| ATC_RAT_RLL_P | 0.07 |
| ATC_RAT_PIT_P | 0.07 |
| ATC_RAT_RLL_I | 0.06 |
| ATC_RAT_PIT_I | 0.06 |
| ATC_RAT_RLL_D | 0.0045 |
| ATC_RAT_PIT_D | 0.0045 |

**Yaw**

| Parameter | Value |
|-----------|-------|
| ATC_RAT_YAW_P | 0.05 |
| ATC_RAT_YAW_I | 0.015 |
| ATC_RAT_YAW_D | 0.0 |

---

## Notes

- Most stable hover performance observed with the January 28th commit.

---

## Mapped Environment

![Autonomously mapped environment](image%20(9).png)

---

## Build

### Compile Command
```bash
riscv64-linux-gnu-gcc -Os -std=gnu11 -w -static \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -I"$HOME/c_library_v2" \
  clean_uav_fc_tof_nav.c -o uav_fc_tof_nav_riscv_static -lm
```
