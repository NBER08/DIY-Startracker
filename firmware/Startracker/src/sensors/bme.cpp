#include "bme.h"

void bme_begin() {
    bme.begin();
}

BmeData bme_read() {
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    BmeData data;
    data.temperature = temp_event.temperature;
    data.pressure = pressure_event.pressure;
    data.humidity = humidity_event.relative_humidity;

    return data;
}