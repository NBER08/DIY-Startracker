#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

typedef struct {
    int64_t temperature;
    int64_t pressure;
    int64_t humidity;
    int64_t dewpoint;      // calculated dew point temperature
    bool  dew_warning;     // true if temp < dewpoint + 2°C
} BmeData;

void bme_begin();

BmeData bme_read();