#!/usr/bin/env python3
"""
validate_sidereal_rate.py

Independent validation of the sidereal step rate calculations defined in
firmware/include/config.h.

Run this script to verify that your gear ratio / microstepping combination
achieves the required angular resolution and generates a step frequency the
ESP32 timer can realistically hit.

Usage:
    python scripts/validate_sidereal_rate.py
"""

import math

# ─── Mirror the constants from config.h ──────────────────────────────────────
SIDEREAL_DAY_SEC    = 86164.0905
MOTOR_STEPS_REV     = 200
MICROSTEP_FACTOR    = 64
GEAR_RATIO          = 144.0

# ─── Derived values ───────────────────────────────────────────────────────────
steps_per_output_rev = MOTOR_STEPS_REV * MICROSTEP_FACTOR * GEAR_RATIO
deg_per_step         = 360.0 / steps_per_output_rev
arcsec_per_step      = deg_per_step * 3600.0

step_interval_us     = (SIDEREAL_DAY_SEC * 1e6) / steps_per_output_rev
step_freq_hz         = 1e6 / step_interval_us

# ─── Drift analysis (crystal accuracy) ───────────────────────────────────────
crystal_ppm          = 20   # typical ESP32-S3 crystal accuracy
freq_error_hz        = step_freq_hz * crystal_ppm * 1e-6
angular_error_per_hr_arcsec = freq_error_hz * 3600 * arcsec_per_step

# ─── Focal length check ───────────────────────────────────────────────────────
def max_exposure_sec(focal_length_mm: float, sensor_pixel_size_um: float = 4.3) -> float:
    """
    Estimate maximum untracked exposure before one-pixel trailing.
    Uses the sidereal rate at the equator (worst case).
    sensor_pixel_size_um: pixel pitch in microns (default: Sony IMX571, APS-C)
    """
    plate_scale_arcsec_px = 206.265 * sensor_pixel_size_um / focal_length_mm
    sidereal_rate_arcsec_s = 15.041  # arcseconds per second at equator
    return plate_scale_arcsec_px / sidereal_rate_arcsec_s

# ─── Output ───────────────────────────────────────────────────────────────────
print("=" * 60)
print("  StarTracker — Sidereal Rate Validation")
print("=" * 60)
print(f"\n  Motor:        {MOTOR_STEPS_REV} steps/rev")
print(f"  Microstepping: 1/{MICROSTEP_FACTOR}")
print(f"  Gear ratio:    1:{GEAR_RATIO:.0f}")
print(f"\n  Steps/output rev:    {steps_per_output_rev:,.0f}")
print(f"  Resolution:          {arcsec_per_step:.4f} arcsec/step  ({deg_per_step*60:.5f} arcmin/step)")
print(f"\n  Step interval:       {step_interval_us:.1f} µs")
print(f"  Step frequency:      {step_freq_hz:.3f} Hz")
print(f"\n  Crystal drift ({crystal_ppm} ppm): {angular_error_per_hr_arcsec:.2f} arcsec/hr drift")

print("\n  Max untracked exposure before 1-pixel trailing:")
for fl in [24, 50, 85, 135, 200, 400]:
    t = max_exposure_sec(fl)
    print(f"    {fl:>4d} mm: {t:.1f}s  (tracking error budget: {arcsec_per_step:.3f} arcsec)")

# ─── Pass/fail checks ─────────────────────────────────────────────────────────
print("\n  Checks:")
checks = [
    ("Angular resolution < 1 arcsec/step",   arcsec_per_step < 1.0),
    ("Step frequency > 1 Hz (not too slow)",  step_freq_hz > 1.0),
    ("Step frequency < 50000 Hz (ESP32 safe)", step_freq_hz < 50000.0),
    ("Crystal drift < 10 arcsec/hr",          angular_error_per_hr_arcsec < 10.0),
]
all_pass = True
for label, result in checks:
    status = "✅ PASS" if result else "❌ FAIL"
    print(f"    {status}  {label}")
    if not result:
        all_pass = False

print()
if all_pass:
    print("  ✅ All checks passed — this configuration is viable.")
else:
    print("  ❌ One or more checks failed — adjust GEAR_RATIO or MICROSTEP_FACTOR.")
print()
