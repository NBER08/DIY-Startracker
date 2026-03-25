#include "lora.h"
#include "protocol.h"
#include "tui.h"
#include <pthread.h>
#include <stdlib.h>


static void on_command(const char* input) {
    char cmd[32];
    float arg1 = 0, arg2 = 0;
    int n = sscanf(input, "%31s %f %f", cmd, &arg1, &arg2);

    if      (strcmp(cmd, "start")   == 0) { lora_cmd_start();  tui_log("CMD_START sent"); }
    else if (strcmp(cmd, "stop")    == 0) { lora_cmd_stop();   tui_log("CMD_STOP sent"); }
    else if (strcmp(cmd, "ping")    == 0) { lora_cmd_ping();   tui_log("Ping sent"); }
    else if (strcmp(cmd, "shutter") == 0) { lora_cmd_shutter((uint8_t)arg1); tui_log("Shutter %.0fms", arg1); }
    else if (strcmp(cmd, "slew")    == 0) { lora_cmd_slew_az((uint16_t)(arg1 * 10)); tui_log("Slew to %.1f°", arg1); }
    else if (strcmp(cmd, "quit")    == 0) { tui_deinit(); exit(0); }
    else if (strcmp(cmd, "help")    == 0) {
        tui_log("Commands: start  stop  ping  shutter <ms>  slew <az>  quit");
    }
    else {
        tui_log("Unknown command: %s", cmd);
    }
}

// ── receive thread ───────────────────────────────────────────────────────────

static void* recv_thread(void* arg) {
    (void)arg;
    LoraStatus status;
    while (1) {
        if (lora_recv_status(&status, 5000) == 0) {
            tui_update_status(&status);
            tui_log("Status OK — batt %.0fmV  %d sats",
                    status.battery_mv, status.gps_sattelites);
        } else {
            tui_log("No status received (timeout)");
        }
    }
    return NULL;
}


LoraConfig_t cfg = {
    .frequency_hz = "radio set freq 868100000\r\n",
    .spreading_factor = "radio set sf sf7\r\n",
    .bandwidth_hz = "radio set bw 125\r\n",
    .tx_power_dbm = "radio set pwr 5\r\n",
    .coding_rate = "radio set cr 4/5\r\n",
    .sync_word = "radio set sync 34\r\n"
};

void main(void) {
    lora_init("/dev/pts/3", &cfg, true);

    tui_init();
    tui_log("LoRa initialized — port /dev/ttyACM0");

    pthread_t rx;
    pthread_create(&rx, NULL, recv_thread, NULL);

    LoraPacket_t pkt;

    tui_run(on_command);   // blocks forever, handles input + redraws
   
    tui_deinit();

}
