#pragma once
#include <libserialport.h>
#include "lora_hal.h"

static struct sp_port* port = NULL;
static int8_t last_rssi = 0, last_snr = 0;

void lora_init(const char* port, uint32_t baud, const LoraConfig_t* cfg) {
    // int fd = open(port, O_RDWR | O_NOCTTY);
	// if (fd < 0) { perror("open"); return 1; }

    // // Configure it
	// struct termios tty;
	// tcgetattr(fd, &tty);
	// cfsetspeed(&tty, baud);
	// tty.c_cflag = CS8 | CREAD | CLOCAL;  // 8N1, no flow control
	// tty.c_iflag = 0;
	// tty.c_oflag = 0;
	// tty.c_lflag = 0;
	// tty.c_cc[VMIN]  = 0;   // non-blocking read
	// tty.c_cc[VTIME] = 10;  // 1 second read timeout (in tenths of a second)
	// tcsetattr(fd, TCSANOW, &tty);

    for (i = 0; i< sizeof(cfg->payload); i++) {
        const char* msg = cfq[i];
        write(fd, msg, strlen(msg));
        Serial.println(msg);
    }
}