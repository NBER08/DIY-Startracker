#pragma once
#include <stdbool.h>

typedef struct {
    double lat;
    double lon;
    long unix_sec;
    int sattelites;
    bool valid;
} GpsFix;

void gps_begin();

GpsFix gps_read();