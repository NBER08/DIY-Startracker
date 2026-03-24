#include "lora.h"
#include "protocol.h"

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

    LoraPacket_t pkt;

    while (1) {
    // Try to receive a packet (wait up to 5 seconds)
    if (lora_recv(&pkt, 5000) == 0) {
        printf("Packet received!\n");
        printf("  RSSI     : %d dBm\n", pkt.rssi);
        printf("  SNR      : %d dB\n",  pkt.snr);
        printf("  Length   : %d bytes\n", pkt.payload_len);
        printf("  Payload  : ");
        for (int i = 0; i < pkt.payload_len; i++)
            printf("%02X ", pkt.payload[i]);
        printf("\n");

        // If you want to interpret it as a LoraStatus struct:
        if (pkt.payload_len == sizeof(LoraStatus)) {
            LoraStatus status;
            memcpy(&status, pkt.payload, sizeof(LoraStatus));
            printf("  Tracking : %s\n",   status.is_tracking ? "yes" : "no");
            printf("  Battery  : %.0f mV\n", status.battery_mv);
            printf("  GPS sats : %d\n",   status.gps_sattelites);
            printf("  RSSI     : %d\n",   status.rssi);
        }

        } else {
        // Timeout — no packet in 5s, send a ping
        printf("No packet received, sending ping...\n");
        lora_send("PING");
        }
    }    
}
