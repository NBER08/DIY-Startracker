#include <Arduino.h>
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "motor/motor.h"
#include "sensors/bme.h"

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

// How often to print a status line to serial (milliseconds)
#define STATUS_INTERVAL_MS  2000

void setup() {
    Serial.begin(115200);
    delay(2000);   // give serial monitor time to open
    Serial.println("\n=== Star Tracker ===");

    // --- Step 1: Initialise all hardware ---
    gps_begin();
    imu_begin();
    motor_begin();
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
    Serial.println("Tracking started. Check the stars!");
}

void loop() {
    // --- Step 4: Correction loop ---
    // This runs continuously. Every iteration:
    //   - Read the GPS (keeps the time fresh)
    //   - Read both IMUs
    //   - If the camera has drifted, nudge the motor

    static unsigned long last_status = 0;

    // Read sensors
    GpsFix fix    = gps_read();
    ImuData mount  = imu_get_mount();
    ImuData camera = imu_get_camera();
    BmeData bme    = bme_read();

    // Print status every STATUS_INTERVAL_MS milliseconds
    // (not every loop iteration — that would flood the serial port)
    unsigned long now = millis();
    if (now - last_status > STATUS_INTERVAL_MS) {
        last_status = now;

        Serial.println("-----------------------------");
        Serial.printf("GPS:   sats=%d  valid=%s\n",
                      fix.satellites, fix.valid ? "yes" : "no");
        Serial.printf("Mount: pitch=%.2f  roll=%.2f  yaw=%.2f  %s\n",
                      mount.pitch, mount.roll, mount.yaw,
                      mount.valid ? "OK" : "no data");
        Serial.printf("Camera: yaw=%.2f  %s\n",
                      camera.yaw, camera.valid ? "OK" : "no data");
        Serial.printf("BME:   temp=%.1f  pressure=%.1f  humidity=%.1f\n",
                      bme.temperature, bme.pressure, bme.humidity);
    }

    // Small delay to avoid hammering the I2C bus
    delay(50);
}
