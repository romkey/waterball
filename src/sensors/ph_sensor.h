#pragma once

#include <Adafruit_ADS1015.h>

#include "sensor.h"

class PH_Sensor : public Sensor {
 public:
  PH_Sensor(uint8_t input, uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated), _input(input) {};

  void begin();
  void handle();

  bool valid() { return _ph > 0 && _ph < 14; };
  float ph() { _mark_read(); return _ph; };

 private:
  Adafruit_ADS1115 _ads;
  uint8_t _input;

  float _ph;
  float _slope;
  float _offset;
};
