# StarTracker

An autonomous LoRa-controlled star tracker for astrophotography, built around a custom PCB and 3D-printed equatorial mount.

## Features

- **Auto polar alignment** — GPS-derived position + time, magnetometer-based north finding, automatic slew to celestial pole
- **Silent sidereal tracking** — TMC2209 stepper drivers with microstepping; near-zero vibration during exposures
- **Wireless control** — LoRa (868 MHz EU) remote from phone or companion board; no cables touching the tracker mid-session
- **IMU-assisted alignment refinement** — ICM-42688-P gyroscope detects polar axis wobble for software drift alignment
- **Compact & field-ready** — 6V input, 3D-printed PETG body, tripod mount

## Hardware Summary

| Subsystem | Part |
|---|---|
| MCU | ESP32-S3-WROOM-2 |
| Stepper drivers | TMC2209, two TB6612FNG |
| IMU | BNO085 |
| GPS | NEO-M8N module |
| Magnetometer | IISMDCTR |
| Power sense sensor| two INA219 |
| LoRa | WLR089U (868 MHz) |
| Power input | 6V DC from a UPS battery |

## Description

### Firmware



### PCB

The PCB was made by me in KICAD 9. 
The main goal of the PCB is to precisely drive the motors to achieve the needed precision (especially the polar tracker motor)
It also includes sensors for orientation, and measuring battery voltage and current and a LoRa module for remote communication and a connector for external modules.

### Mechanical



## Critical Research Items (Pre-Build Checklist)



## License

Hardware and mechanical designs: [CERN-OHL-S v2](https://ohwr.org/cern_ohl_s_v2.txt)  
Firmware: [MIT](LICENSE)
