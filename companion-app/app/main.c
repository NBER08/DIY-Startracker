#include "lora.h"

LoraConfig_t cfg = {
    .frequency_hz = "radio set freq 868100000\r\n",
    .spreading_factor = "radio set sf sf7\r\n",
    .bandwidth_hz = "radio set bw 125\r\n",
    .tx_power_dbm = "radio set pwr 5\r\n",
    .coding_rate = "radio set cr 4/5\r\n",
    .sync_word = "radio set sync 34\r\n"
};

void main(void) {
    lora_init("/dev/ttyACM0", &cfg, false);

    while (1) {
        lora_last_rssi();
        lora_last_snr();
        lora_send("hello");
        sleep(10);
    }
}
