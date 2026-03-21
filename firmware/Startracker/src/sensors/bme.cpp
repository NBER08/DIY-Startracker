#include "bme.h"

static Adafruit_BME280 bme;

static BmeData latest_bme = {};

// =============================================================================
//  Dew point calculation — Magnus formula
//
//  The dew point is the temperature at which water starts to condense.
//  If the air temperature falls below the dew point, you get dew (or frost).
//  On a camera lens this looks like a fog that ruins your images.
//
//  The Magnus formula gives a good approximation:
//    γ = (b × T / (c + T)) + ln(RH/100)
//    Td = c × γ / (b - γ)
//  where b=17.67, c=243.5°C, T=temperature, RH=relative humidity
// =============================================================================

static float calculate_dewpoint(float temp_c, float humidity_rh) {
    const float b = 17.67f;
    const float c = 243.5f;
    float gamma = (b * temp_c / (c + temp_c)) + log(humidity_rh / 100.0f);
    return c * gamma / (b - gamma);
}

void bme_begin() {
    bme.begin();

    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF,
        Adafruit_BME280::STANDBY_MS_1000
    );
}

BmeData bme_read() {
    latest_bme.temp = bme.readTemperature();
    latest_bme.press = bme.readPressure() / 100.0f; // Pa -> hPa
    latest_bme.hum = bme.readHumidity();
    latest_bme.dewpoint = calculate_dewpoint(latest_bme.temp, latest_bme.hum);
    latest_bme.hum_warning = (latest_bme.temp < latest_bme.dewpoint + 2.0f);
    return latest_bme;
}