#include "config.h"
#include "hw.h"

#include "bme280.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

static Adafruit_BME280 bme280;

bool bme280_setup() {
  //  return bme280.begin(BME280_ADDRESS);
  if(!bme280.begin(0x76))
    return false;

  bme280.setSampling(Adafruit_BME280::MODE_NORMAL,
		   Adafruit_BME280::SAMPLING_X2,
		   Adafruit_BME280::SAMPLING_X16,
		   Adafruit_BME280::SAMPLING_X1,
		   Adafruit_BME280::FILTER_X16,
		   Adafruit_BME280::STANDBY_MS_0_5);

  return true;
}

void bme280_loop() {
}

float bme280_temperature() {
  return bme280.readTemperature();
}

float bme280_humidity() {
  return bme280.readHumidity();
}


float bme280_pressure() {
  return bme280.readPressure() / 100.0;
}

