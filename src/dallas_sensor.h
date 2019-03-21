#pragma once

#include "sensor.h"

class Dallas_Sensor : public Sensor {
 public:
  Dallas_Sensor(uint8_t pin, DeviceAddress address, uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) {};

  void begin();
  void handle();

  uint16_t temperature() { _mark_read(); return _temperature; };
  uint16_t humidity() { _mark_read(); return _humidity; };
  uint16_t pressure() { _mark_read(); return _pressure; };
  uint16_t altitude() { _mark_read(); return _altitude; };

 private:
  OneWire _one_wire;
  DallasTemperature _sensors;

  DeviceAddress _address;
  uint8_t _pin;
};
