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
    .is_tracking = false,
    .gps_sattelites = 0,
    .temp = 0,
    .hum = 0,
    .hum_warning = false,
    .pole_az_deg = 0,
    .current_az_deg = 0,
    .alt_correction_deg = 0,
    .current_camera_az = 0,
    .current_camera_alt = 0,
    .is_on_target = false,
    .battery_mv = 0,
    .current_ma = 0,
    .rssi = 1
};

int main(int argc, char** argv) {
    lora_init("/dev/ttyACM0", &cfg, false);
    lora_send_status(&status);
    return 0;
}