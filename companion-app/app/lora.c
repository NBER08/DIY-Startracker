#include "lora.h"
#define SERIAL_RX_BUF_SIZE  256  // max message size

static int8_t last_rssi = 0, last_snr = 0;
static int fd = 3;

static char  rx_buf[SERIAL_RX_BUF_SIZE];
static size_t rx_pos = 0;

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

void on_lora_message(const char* msg) {
  if (strncmp(msg, "radio_rx ", 9) == 0) {
    // Parse RSSI/SNR from "radio_rx 123 456 ..." (your LoRa module format)
    sscanf(msg + 9, "%hhd %hhd", &last_rssi, &last_snr);
    printf("RX: RSSI=%d SNR=%d\n", last_rssi, last_snr);
    // Trigger your action: e.g., process payload
  } else if (strstr(msg, "CRC_ERROR")) {
    // Handle error
  }
}

void serialCheck(int fd) {
  char buf[32];  // small read chunk
  ssize_t n;

  // Read available bytes non-blockingly (VMIN=0 already set)
  while ((n = read(fd, buf, sizeof(buf))) > 0) {
    for (ssize_t i = 0; i < n; i++) {
      char c = buf[i];

      // Complete message: \r\n
      if (rx_pos >= 2 && rx_buf[rx_pos-2] == '\r' && c == '\n') {
        rx_buf[rx_pos-2] = '\0';  // strip \r\n, null-terminate
        printf("< %s\n", rx_buf);  // your debug print
        on_lora_message(rx_buf);   // CALLBACK: do something!
        rx_pos = 0;                // reset
      } else {
        // Accumulate
        if (rx_pos < SERIAL_RX_BUF_SIZE - 1) {
          rx_buf[rx_pos++] = c;
        } else {
          // Overflow: reset (or log error)
          rx_pos = 0;
        }
      }
    }
  }
  // n <= 0: no data or error, do nothing (non-blocking)
}

void lora_init(const char* port, const LoraConfig_t* cfg, bool config) {
	int fd = open(port, O_RDWR | O_NOCTTY);
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