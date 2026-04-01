#include "lora.h"
#include <stdlib.h>

LoraConfig_t cfg = {
    .frequency_hz = "radio set freq 868100000\r\n",
    .spreading_factor = "radio set sf sf7\r\n",
    .bandwidth_hz = "radio set bw 125\r\n",
    .tx_power_dbm = "radio set pwr 5\r\n",
    .coding_rate = "radio set cr 4/5\r\n",
    .sync_word = "radio set sync 34\r\n"
};

LoraStatus_t status = {
    .is_tracking = true,
    .gps_sattelites = 6,
    .temp = 212,
    .hum = 101,
    .hum_warning = true,
    .pole_az_deg = 20.1,
    .current_az_deg = 34.2,
    .alt_correction_deg = 22.8,
    .current_camera_az = 12.8,
    .current_camera_alt = 29.1,
    .is_on_target = true,
    .battery_mv = 100.1,
    .current_ma = 100.1,
    .rssi = 1
};

int main(int argc, char** argv) {
    lora_init("/dev/ttyACM0", &cfg, false);
    lora_send_status(&status);
    return 0;
}
