#pragma once

#include <Arduino.h>

class Relay {
public:
  Relay(uint8_t pin) : _pin(pin) {};

  void begin() { pinMode(_pin, OUTPUT); };
  void on() { digitalWrite(_pin, HIGH); };
  void off() { digitalWrite(_pin, LOW); };
private:
  uint8_t _pin;
};
