#include "tcs34725_sensor.h"

void TCS34725_Sensor::begin() {
  _tcs34725 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

  if(!_tcs34725.begin()) {
    Serial.println("Could not find a valid TCS34725 sensor, check wiring!");
    _status = SENSOR_NOT_FOUND;
    return;
  }
}

void TCS34725_Sensor::handle() {
  uint16_t c;

  _tcs34725.getRawData(&_red, &_green, &_blue, &_color_temperature);

  // colorTemp = _tcs34725.calculateColorTemperature(r, g, b);
  //  _color_temperature = _tcs34725.calculateColorTemperature_dn40(_red, _green, _blue, c);
  _lux = _tcs34725.calculateLux(_red, _green, _blue);
}
