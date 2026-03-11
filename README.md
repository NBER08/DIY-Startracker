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
| MCU | ESP32-S3-MINI-1 |
| Stepper drivers | TMC2209 (×3, UART mode) |
| IMU | ICM-42688-P (SPI) |
| GPS | ATGM336H / L76K module |
| Magnetometer | QMC5883L (I²C) |
| LoRa | Ra-01S (SX1276, 868 MHz) |
| Power input | 12V DC, buck-converted to 3.3V logic |

## Repository Layout

```
startracker/
├── firmware/          # PlatformIO ESP32-S3 project
│   ├── src/           # Application source (.cpp)
│   ├── include/       # Shared headers
│   └── lib/           # Vendored / local libraries
├── hardware/          # KiCad 9 PCB project
│   └── startracker/
├── mechanical/        # 3D printable parts
│   ├── printed-parts/ # STL exports (print-ready)
│   └── cad-source/    # FreeCAD / STEP source files
├── companion-app/     # LoRa remote control application
├── docs/              # Design notes, datasheets, calibration guides
└── scripts/           # Python utility scripts (astro math, calibration)
```

## Getting Started

### Firmware

Requires [PlatformIO](https://platformio.org/) (VS Code extension recommended).

```bash
cd firmware
pio run                  # Build
pio run -t upload        # Flash via USB
pio device monitor       # Serial console (115200 baud)
```

### Hardware

Open `hardware/startracker/startracker.kicad_pro` in KiCad 9.

### Mechanical

STL files in `mechanical/printed-parts/` are print-ready. Recommended settings:
- Material: PETG or ASA (cold-weather stability)
- Layer height: 0.15 mm for gear surfaces, 0.20 mm elsewhere
- Infill: 40%+ for structural parts, 20% for covers

## Critical Research Items (Pre-Build Checklist)

See [`docs/RESEARCH_CHECKLIST.md`](docs/RESEARCH_CHECKLIST.md) for the six items that must be validated before finalising the design.

## License

Hardware and mechanical designs: [CERN-OHL-S v2](https://ohwr.org/cern_ohl_s_v2.txt)  
Firmware: [MIT](LICENSE)
