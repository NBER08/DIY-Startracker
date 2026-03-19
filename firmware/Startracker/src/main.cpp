#include <Arduino.h>
#include "config.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/bme.h"
#include "motor/motor.h"
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

static float get_temperature()  { return 14.5f;    }
static float get_humidity()     { return 68.0f;    }
static float get_battery_mv()   { return 11800.0f; }
static float get_current_ma()   { return 420.0f;   }

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

    // --- Step 3: Start tracking ---
    // At this point we have GPS (so we know where we are and what time it is).
    // The motor knows its own sidereal rate from the compile-time constants.
    // So we just start it — it will track at the correct rate immediately.
    //
    // In the full version, we would also:
    //   - Auto-align the polar axis using the magnetometer
    //   - start reading from sensors and start LoRa telemetry
    // For now we assume you've manually pointed the polar axis at Polaris.

    motor_start_tracking();
    is_tracking = true;
    Serial.println("Tracking started. Check the stars!");
}

void loop() {
    // --- Step 4: Correction loop ---
    // This runs continuously. Every iteration:
    //   - Read the GPS (keeps the time fresh)
    //   - Read both IMUs
    //   - If the camera has drifted, nudge the motor

    static unsigned long last_status = 0;
    static unsigned long last_lora   = 0;

    GpsFix  fix    = gps_read();
    ImuData mount  = imu_get_mount();
    ImuData camera = imu_get_camera();

    // Read sensors
    GpsFix fix    = gps_read();
    ImuData mount  = imu_get_mount();
    ImuData camera = imu_get_camera();
    BmeData bme    = bme_read();

        // LoRa command handler
    LoraCmdPacket pkt = lora_get_command();
    switch (pkt.cmd) {
        case CMD_START:
            if (!is_tracking) {
                motor_start_tracking();
                is_tracking = true;
            }
            break;
        case CMD_STOP:
            if (is_tracking) {
                motor_stop();
                is_tracking = false;
            }
            break;
        case CMD_SHUTTER:
            // pkt.param = exposure duration in seconds sent by the controller
            trigger_shutter(pkt.param);
            break;
        case CMD_PING:
            Serial.println("LoRa: ping received");
            break;
        default:
            break;
    }

    // Serial status
    unsigned long now = millis();
    if (now - last_status > STATUS_INTERVAL_MS) {
        last_status = now;
        Serial.printf("steps=%llu  err=%.4f  sats=%d  temp=%.1fC  hum=%.1f%%  bat=%.2fV  %dma\n",
                      (unsigned long long)motor_get_step_count(),
                      fix.satellites,
                      get_temperature(),
                      get_humidity(),
                      get_battery_mv() / 1000.0f,
                      (int)get_current_ma());
    }

    // LoRa status broadcast
    if (now - last_lora > LORA_INTERVAL_MS) {
        last_lora = now;
        float hum = get_humidity();
        LoraStatus s;
        s.tracking_error    = 0;
        s.is_tracking       = is_tracking;
        s.gps_satellites    = fix.satellites;
        s.gps_valid         = fix.valid;
        s.temperature_c     = get_temperature();
        s.humidity_pct      = hum;
        s.humidity_warning  = (hum > HUMIDITY_WARN_PCT);
        s.battery_mv        = get_battery_mv();
        s.current_ma        = get_current_ma();
        s.rssi              = 0;
        lora_send_status(s);
    }

    // Small delay to avoid hammering the I2C bus
    delay(50);
}
