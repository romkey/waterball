#include "sonar_sensor.h"

void Sonar_Sensor::begin() {
  pinMode(_trigger_pin, OUTPUT);
  pinMode(_echo_pin, INPUT);
}

void Sonar_Sensor::handle() {
  digitalWrite(_trigger_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);

  unsigned duration = pulseIn(_echo_pin, HIGH);
  Serial.printf("sonar duration %u\n", duration);

  _distance_mm = duration*0.34/2;
  //  _distance_cm = duration*0.034/2;
  _distance_cm = duration/10;
}
