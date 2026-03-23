#include "lora.h"

void main(void) {
    lora_init("/dev/ttyACM0", 115200, &cfg);
    lora_deinit();
}