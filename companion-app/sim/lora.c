#include "lora.h"
#include <stdlib.h>
#include <sys/time.h> 


int fd = 3;

static void read_response(int fd) {
    char buf[256];
    int pos = 0;
    ssize_t n;
    while (pos < (int)sizeof(buf) - 1) {
        n = read(fd, buf + pos, 1);
        if (n <= 0) break;
        pos++;
        if (pos >= 2 && buf[pos-2] == '\r' && buf[pos-1] == '\n') {
            buf[pos] = '\0';
            printf("< %s", buf);
            pos = 0;
        }
    }
}

void bytes_to_hex(const uint8_t* in, size_t len, char* out) {
    for (size_t i = 0; i < len; i++) {
        sprintf(out + i*2, "%02X", in[i]);
    }
    out[len*2] = '\0';
}

static int send_cmd(int fd, const char* value) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf), value);
    if (write(fd, buf, len) < 0) { perror("write"); return -1; }
    printf("\nSent: %s", buf);
	read_response(fd);
    return 0;
}

void lora_send(const uint8_t* payload, size_t len) {
    char hex[2*len + 1];
    char cmd[256];

    bytes_to_hex(payload, len, hex);

    send_cmd(fd, "radio rxstop\r\n");

    snprintf(cmd, sizeof(cmd), "radio tx %s 1\r\n", hex);
    send_cmd(fd, cmd);

    send_cmd(fd, "radio rx 0\r\n");
}

int lora_configure(int fd, const LoraConfig_t* cfg) {
    if (send_cmd(fd,  cfg->frequency_hz)      < 0) return -1;
    if (send_cmd(fd,    cfg->spreading_factor)   < 0) return -1;
    if (send_cmd(fd,    cfg->bandwidth_hz)       < 0) return -1;
    if (send_cmd(fd,   cfg->tx_power_dbm)       < 0) return -1;
    if (send_cmd(fd,    cfg->coding_rate)        < 0) return -1;
    if (send_cmd(fd, cfg->sync_word)          < 0) return -1;
    return 0;
}

void lora_init(const char* port, const LoraConfig_t* cfg, bool config) {
	fd = open(port, O_RDWR | O_NOCTTY);
	if (fd < 0) { perror("open"); return; }
	printf("%d", fd);
    // Configure it
	struct termios tty;
	tcgetattr(fd, &tty);
	cfsetspeed(&tty, B115200);
	tty.c_cflag = CS8 | CREAD | CLOCAL;  // 8N1, no flow control
	tty.c_iflag = 0;
	tty.c_oflag = 0;
	tty.c_lflag = 0;
	tty.c_cc[VMIN]  = 0;   // non-blocking read
	tty.c_cc[VTIME] = 30;  // 1.5 second read timeout (in tenths of a second)
	tcsetattr(fd, TCSANOW, &tty);
	send_cmd(fd, "sys reset\r\n");
    if  (config){
	send_cmd(fd, "sys reset\r\n");
	send_cmd(fd, "sys get ver\r\n");
    	lora_configure(fd, cfg);
    }
	send_cmd(fd, "radio rx 0\r\n");
}

void lora_send_status(LoraStatus_t *s) {
    uint8_t buf[24] = {};

    buf[0] = 0xAB; // magic

    // Flags:
    buf[1] = (s->is_tracking        ? 0x01 : 0)  //0000000x
            |(s->is_on_target       ? 0x02 : 0)  //000000x0
            |(s->hum_warning        ? 0x04 : 0); //00000x00
            
    buf[2] = (uint8_t)s->gps_sattelites;
    // Temperature — degrees × 10, signed 16-bit
    int16_t temp = (int16_t)(s->temp * 10.0f);
    buf[3] = temp >> 8;  buf[4] = temp & 0xFF;
    // Humidity — percent × 10
    uint16_t hum = (uint16_t)(s->hum * 10.0f);
    buf[5] = hum >> 8;  buf[6] = hum & 0xFF;

    // Pole azimuth — degrees × 10, unsigned 16-bit
    uint16_t paz = (uint16_t)(s->pole_az_deg * 10.0f);
    buf[7] = paz >>8; buf[8] = paz & 0xFF;

    // Current platform azimuth — degrees × 10
    uint16_t caz = (uint16_t)(s->current_az_deg * 10.0f);
    buf[9] = caz >> 8;  buf[10] = caz & 0xFF;

    // Altitude correction — degrees × 10, signed 16-bit
    // positive = tilt up, negative = tilt down
    int16_t alt_corr = (int16_t)(s->alt_correction_deg * 10.0f);
    buf[11]  = alt_corr >> 8;  buf[12] = alt_corr & 0xFF;

    // Current camera azimuth — degrees × 10
    uint16_t ccaz = (uint16_t)(s->current_camera_az * 10.0f);
    buf[13] = ccaz >> 8;  buf[14] = ccaz & 0xFF;

    // Current camera altitude — degrees × 10
    uint16_t ccal = (uint16_t)(s->current_camera_alt * 10.0f);
    buf[15] = ccal >> 8;  buf[16] = ccal & 0xFF;

    // Battery voltage
    uint16_t batt_v = (uint16_t)(s->battery_mv);
    buf[17] = batt_v >> 8;  buf[18] = batt_v & 0xFF;

    // Power usage in mA
    uint16_t current_ma = (uint16_t)(s->current_ma);
    buf[19] = current_ma >> 8;  buf[20] = current_ma & 0xFF;

    // RSSI of last received packet
    buf[21] = 0x00;
    buf[22] = 0x01;

    // buf[23] reserved

    lora_send((uint8_t*)buf, sizeof(buf));
}
