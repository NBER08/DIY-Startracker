#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

typedef struct {
    int64_t temp;
    int64_t press;
    int64_t hum;
    int64_t dewpoint;      // calculated dew point temperature
    bool hum_warning;
} BmeData;

void bme_begin();
    
BmeData bme_read();