#pragma once

#include "sensor.h"

class Sonar_Sensor : public Sensor {
 public:
  Sonar_Sensor(uint8_t trigger_pin, uint8_t echo_pin, uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated), _trigger_pin(trigger_pin), _echo_pin(echo_pin) {};

  void begin();
  void handle();

  bool valid() {  return (_distance_cm < 100); };
  uint16_t distance_cm() { _mark_read(); return _distance_cm; };
  uint16_t distance_mm() { _mark_read(); return _distance_mm; };

 private:
  uint8_t _trigger_pin;
  uint8_t _echo_pin;

  uint16_t _distance_cm;
  uint32_t _distance_mm;
};
