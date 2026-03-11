# Hardware — KiCad 9

PCB design for the StarTracker main board.

## Opening the project

Open `startracker.kicad_pro` in KiCad 9.

> **Note:** The KiCad project files will be committed once the schematic capture phase begins. This file marks the directory as intentional in the meantime.

## Schematic structure (planned)

The schematic will be split into hierarchical sheets:

| Sheet | Contents |
|---|---|
| `top.kicad_sch` | Top-level block diagram, power flags |
| `mcu.kicad_sch` | ESP32-S3-MINI-1, boot strapping resistors, USB |
| `motors.kicad_sch` | TMC2209 ×3, motor connectors, bulk caps, TVS diodes |
| `sensors.kicad_sch` | ICM-42688-P, QMC5883L, decoupling |
| `gps.kicad_sch` | ATGM336H module header, antenna connector |
| `lora.kicad_sch` | Ra-01S module header, antenna keep-out note |
| `power.kicad_sch` | 12V input, MP2315S buck, filtering, connectors |

## Design rules (target)

- Trace width: 0.2 mm signal, 0.5 mm 3.3V, 1.5 mm motor supply
- Clearance: 0.2 mm minimum
- Via drill: 0.3 mm (0.6 mm pad)
- Board outline: TBD — constrained by mechanical enclosure

## Fabrication

Target: JLCPCB 2-layer, 1.6 mm FR4, HASL finish, green solder mask.  
Assembly: hotplate reflow with solder paste + stencil for QFN/LGA parts.
