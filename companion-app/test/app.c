#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int main(void) {
	int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	if (fd < 0) { perror("open"); return 1; }

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

	const char* msg = "sys reset";
	write(fd, msg, strlen(msg));
	printf("sent: %s\n", msg);

	// Read a response
	char buf[256] = {0};
	int n = read(fd, buf, sizeof(buf) - 1);
	if (n > 0) printf("received: %s\n", buf);

	close(fd);
	return 0;
}
