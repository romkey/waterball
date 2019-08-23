#include "config.h"
#include "hw.h"

#include "tofl.h"

#include <Adafruit_VL53L0X.h>

static Adafruit_VL53L0X tofl;

bool tofl_setup() {
  return tofl.begin();
}

void tofl_loop() {
}

unsigned tofl_distance() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  tofl.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  return measure.RangeMilliMeter;
}
