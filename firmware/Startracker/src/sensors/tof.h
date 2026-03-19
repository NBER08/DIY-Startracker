#pragma once
#include <stdbool.h>

// =============================================================================
//  tof.h — VL53L4CD Time-of-Flight sensor
//
//  Mounted on the base (fixed) platform.
//  Beam points toward a reflective surface on the tilting upper platform.
//
//  As the platform tilts up, the distance increases.
//  We convert that distance into an altitude angle using the geometry
//  defined by TOF_FLAT_MM and TOF_ARM_MM in config.h.
//
//  Geometry (side view):
//
//        hinge
//          |  \
//          |   \  tilting platform
//          |    \
//   TOF ---|---> x  (beam hits here)
//          |
//        base
//
//  TOF_ARM_MM  = horizontal distance from sensor to hinge
//  TOF_FLAT_MM = sensor reading when platform is level (0°)
//
//  When tilted by angle θ:
//    measured = sqrt( TOF_ARM_MM² + (TOF_ARM_MM * tan(θ))² )
//    simplified for small angles: altitude_deg ≈ atan((d - TOF_FLAT_MM) / TOF_ARM_MM)
// =============================================================================

typedef struct {
    float distance_mm;    // raw distance from sensor
    float altitude_deg;   // computed platform altitude angle
    bool  valid;
} TofReading;

void       tof_begin();
TofReading tof_read();
