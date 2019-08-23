#include "config.h"
#include "hw.h"

#include "tofl.h"

#include <OneWire.h>
#include <DallasTemperature.h>

static OneWire oneWire(ONE_WIRE_BUS_PIN);
static DallasTemperature dallas_sensors(&oneWire);
// DeviceAddress sensor1 = { 0x28, 0x3F, 0x0B, 0x77, 0x91, 0x06, 0x02, 0x1C };
// DeviceAddress sensor1 = { 0x28, 0x4B, 0x8, 0x46, 0x92, 0x09, 0x02, 0x2D };
// DeviceAddress sensor1 = { 0x28, 0x3F, 0xB, 0x77, 0x91, 0x6, 0x2, 0x0 };
// DeviceAddress sensor1 = { 0x28, 0xE6, 0xB9, 0x46, 0x92, 0x10, 0x2, 0xE7 };
// static DeviceAddress dallas_sensor1 = { 0x28, 0x28, 0xC8, 0x4D, 0x97, 0x13, 0x03, 0x6A };

bool dallas_setup() {
  dallas_sensors.begin();
  return true;
}

void dallas_loop() {
}

bool dallas_temperature(float* temp) {
  dallas_sensors.requestTemperatures();
  //  temp = dallas_sensors.getTempC(dallas_sensor1);
  *temp = dallas_sensors.getTempCByIndex(0);

  return *temp != DEVICE_DISCONNECTED_C;
}
