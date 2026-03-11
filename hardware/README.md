# Hardware — KiCad 9

PCB design for the StarTracker main board.

## Schematic structure

The schematic will be split into hierarchical sheets:

| Sheet | Contents |
|---|---|
| `Startracker.kicad_sch` | Top-level block diagram, power flags |
| `obc.kicad_sch` | ESP32-S3-WROOM-2 |
| `motor_driver.kicad_sch` | TMC2209, 2x TB6612FNG|
| `sensors.kicad_sch` | BNO085, 2x INA219, IISMDCTR, GPS and external IMU connector|
| `lora.kicad_sch` | WLR089U and programming connector |
| `ps.kicad_sch` | TPS2121RUXR and TPS54308 |
| `usb.kicad_sch` | USB-C receptacle and TVS diode |

## Design rules



## Fabrication

Target: JLCPCB 4-layer, 1.6 mm FR4, HASL finish, green solder mask.  
Assembly: hotplate reflow with solder paste + stencil for QFN/LGA parts.
