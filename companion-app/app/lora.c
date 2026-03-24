#include "lora.h"
#include <sys/time.h> 
#define SERIAL_RX_BUF_SIZE  256  // max message size

static int8_t last_rssi = 0, last_snr = 0;
static int fd = 3;

static char  rx_buf[SERIAL_RX_BUF_SIZE];
static size_t rx_pos = 0;

static uint8_t hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static int read_line(int fd, char* buf, int maxlen, uint32_t timeout_ms) {
    int pos = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (pos < maxlen - 1) {
        // check elapsed time
        gettimeofday(&now, NULL);
        uint32_t elapsed = (now.tv_sec  - start.tv_sec)  * 1000
                         + (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= timeout_ms) return -1;

        char c;
        ssize_t n = read(fd, &c, 1);
        if (n <= 0) continue;   // VTIME handles short blocking, just retry

        buf[pos++] = c;
        if (pos >= 2 && buf[pos-2] == '\r' && buf[pos-1] == '\n') {
            buf[pos-2] = '\0';  // strip \r\n, null-terminate
            return pos - 2;
        }
    }
    return -1;
}

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

static int send_cmd(int fd, const char* value) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf), value);
    if (write(fd, buf, len) < 0) { perror("write"); return -1; }
    printf("\nSent: %s", buf);
	read_response(fd);
    return 0;
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
	tty.c_cc[VTIME] = 10;  // 1 second read timeout (in tenths of a second)
	tcsetattr(fd, TCSANOW, &tty);
	if (config){
	send_cmd(fd, "sys reset\r\n");                       //module needs time to reboot
	send_cmd(fd, "mac pause\r\n"); 

	send_cmd(fd, "sys reset\r\n");
	send_cmd(fd, "sys get ver\r\n");

    lora_configure(fd, cfg);
	}

	send_cmd(fd, "radio rx 0\r\n");
}

void lora_send(const char* payload) {
	send_cmd(fd, "radio rxstop\r\n");
	char txmsg[100];
	char hex_payload[2 * strlen(payload) + 1];
	snprintf(txmsg, sizeof(txmsg), "radio tx %s 1\r\n", hex_payload);
	send_cmd(fd, txmsg);
	send_cmd(fd, "radio rx 0\r\n");
}

int8_t lora_last_rssi(void){
	send_cmd(fd, "radio get rssi\r\n");
	return last_rssi;
}

int8_t lora_last_snr(void){
	send_cmd(fd, "radio get snr\r\n");
	return last_snr;
}

int lora_recv(LoraPacket_t* out, uint32_t timeout_ms) {
    char line[512];

    while (1) {
        int len = read_line(fd, line, sizeof(line), timeout_ms);
        if (len < 0) return -1;   // timed out

        printf("< %s\n", line);

        // Module outputs "radio_rx  <hexdata>"  (two spaces after radio_rx)
        if (strncmp(line, "radio_rx  ", 10) != 0) continue;

        char* hex = line + 10;
        int hex_len = strlen(hex);
        if (hex_len < 2 || hex_len % 2 != 0) return -1;

        // Decode hex string into bytes
        // NOTE: LoraPacket_t.payload is currently a single uint8_t — it can
        // only hold one byte. You should change it to uint8_t payload[255].
        // For now we decode into a local buffer and copy the first byte.
        uint8_t decoded[255];
        int byte_count = hex_len / 2;
        for (int i = 0; i < byte_count && i < 255; i++)
            decoded[i] = (hex_nibble(hex[i*2]) << 4) | hex_nibble(hex[i*2+1]);

        memcpy(out->payload, decoded, byte_count);
        out->payload_len = (uint8_t)byte_count;

        // Get RSSI and SNR immediately after reception while module still holds them
        send_cmd(fd, "radio get rssi\r\n");
        char rssi_line[64];
        if (read_line(fd, rssi_line, sizeof(rssi_line), 500) > 0)
            out->rssi = (int8_t)atoi(rssi_line);

        send_cmd(fd, "radio get snr\r\n");
        char snr_line[64];
        if (read_line(fd, snr_line, sizeof(snr_line), 500) > 0)
            out->snr = (int8_t)atoi(snr_line);

        // Re-arm RX mode so module listens for the next packet
        send_cmd(fd, "radio rx 0\r\n");

        return 0;
    }
}

void lora_send_cmd(LoraCmd cmd, uint8_t param_hi, uint8_t param_lo) {
    uint8_t buf[4] = {
        (uint8_t)cmd,
        param_hi,
        param_lo,
        0x00        // reserved, matches buf[3] on MCU
    };

    // Convert to hex string for the AT-command radio module
    // "radio tx <hex> 1\r\n"
    char hex[9];    // 4 bytes = 8 hex chars + null
    snprintf(hex, sizeof(hex), "%02X%02X%02X%02X",
             buf[0], buf[1], buf[2], buf[3]);

    char txmsg[64];
    snprintf(txmsg, sizeof(txmsg), "radio tx %s\r\n", hex);

    send_cmd(fd, "radio rxstop\r\n");
    send_cmd(fd, txmsg);
    send_cmd(fd, "radio rx 0\r\n");

    printf("TX cmd=0x%02X param_hi=%d param_lo=%d\n", cmd, param_hi, param_lo);
}

// Convenience wrappers
void lora_cmd_start(void)                        { lora_send_cmd(CMD_START,   0, 0); }
void lora_cmd_stop(void)                         { lora_send_cmd(CMD_STOP,    0, 0); }
void lora_cmd_ping(void)                         { lora_send_cmd(CMD_PING,    0, 0); }
void lora_cmd_shutter(uint16_t duration_tenths)   { lora_send_cmd(CMD_SHUTTER, duration_tenths >> 8, duration_tenths & 0xFF); }
void lora_cmd_slew_az(uint16_t angle_tenths)     { lora_send_cmd(CMD_SLEW_AZ, angle_tenths >> 8, angle_tenths & 0xFF); }