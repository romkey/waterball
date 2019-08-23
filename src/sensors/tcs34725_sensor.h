#pragma once

#include "sensor.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>

class TCS34725_Sensor : public Sensor {
 public:
  TCS34725_Sensor(uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) {};

  void begin();
  void handle();

  uint16_t lux() { _mark_read(); return _lux; };
  uint16_t red() { _mark_read(); return _red; };
  uint16_t green() { _mark_read(); return _green; };
  uint16_t blue() { _mark_read(); return _blue; };

 private:
  Adafruit_TCS34725 _tcs34725;

  uint16_t _lux;
  uint16_t _red;
  uint16_t _green;
  uint16_t _blue;
  uint16_t _color_temperature;
};
