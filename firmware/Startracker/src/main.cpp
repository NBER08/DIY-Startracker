#include <Arduino.h>
#include "config.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/bme.h"
#include "sensors/tof.h"
#include "motor/motor.h"
#include "motor/slew.h"
#include "motor/astro.h"
#include "comms/lora.h"


#define SIMULATE_HARDWARE  1

#define STATUS_INTERVAL_MS   2000
#define LORA_INTERVAL_MS     5000
#define ERROR_THRESHOLD_DEG  0.005f
#define HUMIDITY_WARN_PCT    80.0f

static bool is_tracking = false;
static bool    is_slewing_az = false;
static float   target_az     = 0.0f;
static double  target_ra_hrs = 0.0;
static GpsFix  last_fix      = {};

// =========================================================================
//  STAR TRACKER — simplified main program
//
//  The whole program flow is here. Read this top to bottom and you
//  can understand everything the tracker does.
//
//  Flow:
//    1. Start up all hardware
//    2. Wait for GPS fix
//    3. Start the tracking motor
//    4. Loop forever: read IMU, apply small corrections
// =========================================================================


static void trigger_shutter(uint8_t duration_sec) {
    if (duration_sec == 0) duration_sec = 1;
    Serial.printf("Shutter: opening for %d seconds\n", duration_sec);

    digitalWrite(FOCUS_PIN,   HIGH);
    delay(300);                              // focus hold before firing
    digitalWrite(SHUTTER_PIN, HIGH);
    delay((uint32_t)duration_sec * 1000);   // hold open
    digitalWrite(SHUTTER_PIN, LOW);
    digitalWrite(FOCUS_PIN,   LOW);

    Serial.println("Shutter: closed");
}

// ---- Stub sensor reads — replace with real BME280/INA219 calls ----
static float get_temperature() { return 14.5f;    }
static float get_humidity()    { return 68.0f;    }
static float get_battery_mv()  { return 11800.0f; }
static float get_current_ma()  { return 420.0f;   }
static float get_mag_az()      { return 5.0f;     }

static void point_camera_at(double ra_hours, double dec_deg) {
    if (!last_fix.valid) {
        Serial.println("Point: no GPS fix");
        return;
    }

    CameraTarget target = astro_radec_to_motors(
        last_fix.lat, last_fix.lon, last_fix.unix_sec,
        ra_hours, dec_deg
    );

    Serial.printf("Point: RA=%.2fh Dec=%.1f° → HA=%.1f° polar_dist=%.1f°\n",
                  ra_hours, dec_deg, target.ha_deg, target.polar_dist_deg);

    bool was_tracking = is_tracking;
    if (was_tracking) { motor_stop(); is_tracking = false; }

    // Slew polar axis to hour angle position
    uint64_t steps_per_rev = (uint64_t)MOTOR_STEPS * MICROSTEP * GEAR_RATIO;
    int64_t  steps = (int64_t)(target.ha_deg / 360.0 * steps_per_rev);
    motor_slew_steps(steps);

    // Tilt camera to polar distance
    camera_tilt_to((float)target.polar_dist_deg);

    // Verify with BNO085 on camera
    CameraOrientation cam = imu_get_camera();
    if (cam.valid) {
        Serial.printf("Point: camera now at az=%.1f° alt=%.1f°\n",
                      cam.az_deg, cam.alt_deg);
    }

    if (was_tracking) {
        imu_reset_tracking_reference();   // reset drift reference from new position
        motor_start_tracking();
        is_tracking = true;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Star Tracker ===");

    pinMode(SHUTTER_PIN, OUTPUT);
    pinMode(FOCUS_PIN,   OUTPUT);
    digitalWrite(SHUTTER_PIN, LOW);
    digitalWrite(FOCUS_PIN,   LOW);

    gps_begin();
    imu_begin();   // starts Wire — tof_begin() called after so bus is ready
    tof_begin();
    motor_begin();
    slew_begin();
    lora_begin();
    bme_begin();

    // --- Step 2: Wait for GPS fix ---
    Serial.println("Waiting for GPS fix...");

    GpsFix fix;
    while (true) {
        fix = gps_read();

        if (fix.valid) {
            Serial.println("\nGPS fix acquired!");
            Serial.printf("  Latitude:   %.5f\n", fix.lat);
            Serial.printf("  Longitude:  %.5f\n", fix.lon);
            Serial.printf("  Satellites: %d\n",   fix.satellites);
            break;
        }

        // Print a dot every second while waiting
        Serial.print(".");
        delay(1000);
    }

    // Show the user the required platform altitude before starting
    PoleDirection pole = astro_get_pole(last_fix.lat, last_fix.lon, last_fix.unix_sec);
    TofReading    tof  = tof_read();
    Serial.printf("Required platform altitude: %.2f°\n", pole.pole_alt_deg);
    Serial.printf("Current platform altitude:  %.2f° (ToF: %.1f mm)\n",
                  tof.altitude_deg, tof.distance_mm);
    Serial.printf("Adjust platform by hand:    %+.2f°\n",
                  (float)pole.pole_alt_deg - tof.altitude_deg);

    imu_reset_tracking_reference();
    motor_start_tracking();
    is_tracking = true;
    Serial.println("Tracking started.");
}

// -------------------------------------------------------------------------
// loop
// -------------------------------------------------------------------------
void loop() {
    static unsigned long last_status = 0;
    static unsigned long last_lora   = 0;

    last_fix = gps_read();

    // Sensors
    CameraOrientation cam = imu_get_camera();
    TofReading        tof = tof_read();

    // Azimuth slew
    if (is_slewing_az) {
        bool done = slew_az_toward(target_az, get_mag_az());
        if (done) { is_slewing_az = false; Serial.println("Az slew done"); }
    }

    // Pole direction + altitude correction
    PoleDirection pole = {};
    if (last_fix.valid) {
        pole = astro_get_pole(last_fix.lat, last_fix.lon, last_fix.unix_sec);
        // Altitude correction: how far off the ToF-measured angle is from required
        if (tof.valid) {
            pole.alt_correction_deg = (float)pole.pole_alt_deg - tof.altitude_deg;
        }
    }

    // LoRa commands
    LoraCmdPacket pkt = lora_get_command();
    switch (pkt.cmd) {
        case CMD_START:
            if (!is_tracking) {
                imu_reset_tracking_reference();
                motor_start_tracking();
                is_tracking = true;
            }
            break;
        case CMD_STOP:
            if (is_tracking) { motor_stop(); is_tracking = false; }
            break;
        case CMD_SHUTTER:
            trigger_shutter(pkt.param_hi);
            break;
        case CMD_PING:
            Serial.println("LoRa: ping");
            break;
        case CMD_POINT_RA:
            target_ra_hrs = pkt.param_hi + (pkt.param_lo / 60.0);
            Serial.printf("LoRa: target RA=%.3fh\n", target_ra_hrs);
            break;
        case CMD_POINT_DEC: {
            double dec = (int8_t)(pkt.param_hi - 128) + pkt.param_lo / 60.0;
            point_camera_at(target_ra_hrs, dec);
            break;
        }
        case CMD_SLEW_AZ:
            if (last_fix.valid) {
                target_az    = (float)pole.pole_az_deg;
                is_slewing_az = true;
                Serial.printf("LoRa: slewing az → %.1f°\n", target_az);
            }
            break;
        default:
            break;
    }

    unsigned long now = millis();

    // Serial status
    if (now - last_status > STATUS_INTERVAL_MS) {
        last_status = now;
        Serial.printf(
            "tof=%.1fmm plat_alt=%.2f° alt_corr=%+.2f° "
            "cam_az=%.1f° cam_alt=%.1f° "
            "steps=%llu "
            "T=%.1fC H=%.0f%% bat=%.2fV %dma\n",
            tof.distance_mm,
            tof.valid ? tof.altitude_deg : 0.0f,
            pole.alt_correction_deg,
            cam.valid ? cam.az_deg  : 0.0f,
            cam.valid ? cam.alt_deg : 0.0f,
            (unsigned long long)motor_get_step_count(),
            get_temperature(), get_humidity(),
            get_battery_mv() / 1000.0f, (int)get_current_ma()
        );
    }

    // LoRa broadcast
    if (now - last_lora > LORA_INTERVAL_MS) {
        last_lora = now;
        float hum = get_humidity();
        LoraStatus s = {};
        s.is_tracking        = is_tracking;
        s.gps_satellites     = last_fix.satellites;
        s.gps_valid          = last_fix.valid;
        s.pole_az_deg        = (float)pole.pole_az_deg;
        s.current_az_deg     = get_mag_az();
        s.alt_correction_deg = pole.alt_correction_deg;
        s.camera_tilt_deg    = camera_get_tilt_deg();
        s.temperature_c      = get_temperature();
        s.humidity_pct       = hum;
        s.humidity_warning   = (hum > HUMIDITY_WARN_PCT);
        s.battery_mv         = get_battery_mv();
        s.current_ma         = get_current_ma();
        s.rssi               = 0;
        lora_send_status(s);
    }

    delay(50);
}