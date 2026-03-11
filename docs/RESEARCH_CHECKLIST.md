# Research Checklist

Six items that **must be validated before finalising the PCB layout or placing a component order**. Each has a clear pass/fail test defined. Check them off as they are resolved.

---

## 🔴 1 — Gear reduction backlash

**Risk:** A 3D-printed worm gear with even 0.5° of backlash makes the tracker useless.  
**Acceptable threshold:** ≤ 0.05° of backlash at the output shaft (≤ 3 arcminutes).

**Validation method:**
1. Print a prototype worm gear pair in PETG at 0.15 mm layer height.
2. Mount on a rigid test jig. Attach a laser pointer to the output shaft and project onto a wall 3 m away.
3. Push the input shaft CW until contact, mark the laser spot. Push CCW until contact, mark again.
4. Calculate angular error: `arctan(spot_separation_mm / 3000_mm)`.
5. If > 0.05°: switch to GT2 belt + pulley reduction (two-stage), or source a brass worm gear insert.

**Status:** ⬜ Not tested  
**Notes:**

---

## 🔴 2 — TMC2209 thermal performance on home-assembled PCB

**Risk:** The QFN-28 thermal pad makes no contact → chip overheats → thermal shutdown mid-session.  
**Acceptable threshold:** Chip case temperature < 70 °C at full run current after 15 minutes continuous operation.

**Validation method:**
1. Assemble first PCB. Solder TMC2209 with solder paste + stencil on hotplate.
2. Connect motor, set run current to intended value (`MOTOR_POLAR_CURRENT_MA`).
3. Enable motor, run stepper at tracking speed for 15 minutes in a 20 °C ambient.
4. Measure chip temperature with a thermocouple or IR thermometer.
5. If > 70 °C: increase copper pour area around the thermal pad, add vias to inner copper layers, or reduce run current and increase `MICROSTEP_FACTOR`.

**Status:** ⬜ Not tested  
**Notes:**

---

## 🔴 3 — ESP32-S3 timer jitter under full firmware load

**Risk:** FreeRTOS scheduler jitter corrupts the sidereal step pulse timing, causing visible periodic error in photos.  
**Acceptable threshold:** Step pulse timing jitter < ±50 µs (translates to < 0.001° angular error per step at sidereal rate).

**Validation method:**
1. Write a minimal firmware that runs GPS parsing (UART1), IMU SPI reads at 100 Hz, LoRa polling, and the tracking timer ISR simultaneously.
2. Toggle a spare GPIO pin at the start of each step ISR.
3. Capture the GPIO waveform on a logic analyser or oscilloscope.
4. Measure the distribution of inter-pulse intervals. Calculate standard deviation.
5. If jitter > 50 µs: move step pulse generation to a hardware LEDC/PWM peripheral running in isolation from FreeRTOS, or use an external ATtiny85 as a dedicated step clock.

**Status:** ⬜ Not tested  
**Notes:**

---

## 🔴 4 — Magnetometer isolation from stepper motor fields

**Risk:** Energised stepper motors corrupt the magnetometer reading → auto-alignment points in the wrong direction.  
**Acceptable threshold:** Compass heading error < 1° with all motors energised at run current.

**Validation method:**
1. Place the magnetometer at its planned PCB location.
2. Energise all stepper motors at full run current.
3. Compare compass heading reading with motors on vs motors off (use a reference compass or known landmark bearing).
4. If error > 1°: move magnetometer to a daughterboard on a 10 cm ribbon cable, positioned away from all motors. Re-test.
5. If still > 1°: add a mu-metal shield around the magnetometer, or switch to a magnetometer with better noise rejection.

**Status:** ⬜ Not tested  
**Notes:**

---

## 🔴 5 — GPS + LoRa RF coexistence

**Risk:** LoRa TX at +20 dBm desensitises the GPS receiver → loss of fix during field operation.

**Validation method:**
1. On the assembled board, establish a GPS fix. Log fix quality (number of satellites, HDOP).
2. Transmit on LoRa at maximum power (`LORA_TX_POWER_DBM = 20`).
3. Monitor GPS fix quality before, during, and after LoRa TX.
4. If fix is lost or satellite count drops significantly: implement firmware sequencing (do not transmit LoRa while GPS is actively acquiring). Alternatively, add an RF shield wall between the GPS antenna trace and the LoRa module on the PCB.

**Status:** ⬜ Not tested  
**Notes:**

---

## 🔴 6 — Motor torque budget

**Risk:** Insufficient holding torque causes the polar axis to slip under camera + lens weight → star trails.  
**Acceptable threshold:** Motor holds position against maximum planned payload with ≥ 2× safety margin.

**Validation method:**
1. Determine max payload: camera body + heaviest intended lens. Weigh in grams.
2. Measure platform arm length (pivot to camera mount centre) in mm.
3. Calculate required torque: `T = (mass_kg × 9.81 × arm_m)` N·m.
4. Multiply by 2 for safety margin. Compare to motor holding torque spec at your run current setting.
5. Verify NEMA 17 at `MOTOR_POLAR_CURRENT_MA` can meet this. If not: increase current (check TMC2209 thermal headroom) or switch to a NEMA 17 with longer body (higher torque rating).

**Example:** 1.5 kg payload, 150 mm arm → T = 1.5 × 9.81 × 0.15 ≈ **2.2 N·m** ... wait, that's wrong — recalculate with correct geometry. A horizontal arm at the equator requires resisting gravitational torque; at other hour angles the load is partially supported. Measure empirically on the physical prototype.

**Status:** ⬜ Not tested  
**Notes:**

---

*Last updated: initial scaffold*
