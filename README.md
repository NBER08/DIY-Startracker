# StarTracker

An autonomous LoRa-controlled star tracker for astrophotography, built around a custom PCB and 3D-printed equatorial mount.

## Features

- **Auto polar alignment** — GPS-derived position + time, magnetometer-based north finding, automatic slew to celestial pole
- **Silent sidereal tracking** — TMC2209 stepper drivers with microstepping; near-zero vibration during exposures
- **Wireless control** — LoRa (868 MHz EU) remote from phone or companion board; no cables touching the tracker mid-session
- **IMU-assisted alignment refinement** — ICM-42688-P gyroscope detects polar axis wobble for software drift alignment
- **Compact & field-ready** — 12V input (standard astro battery), 3D-printed PETG/ASA body, ⅜"-16 tripod mount

## Hardware Summary

| Subsystem | Part |
|---|---|
| MCU | ESP32-S3-WROOM-2 |
| Stepper drivers | TMC2209,  |
| IMU | ICM-42688-P (SPI) |
| GPS | ATGM336H / L76K module |
| Magnetometer | QMC5883L (I²C) |
| LoRa | Ra-01S (SX1276, 868 MHz) |
| Power input | 12V DC, buck-converted to 3.3V logic |

## Description

### Firmware



### PCB



### Mechanical



## Critical Research Items (Pre-Build Checklist)



## License

Hardware and mechanical designs: [CERN-OHL-S v2](https://ohwr.org/cern_ohl_s_v2.txt)  
Firmware: [MIT](LICENSE)
