#include "ph_sensor.h"

// based on https://scidle.com/how-to-use-a-ph-sensor-with-arduino/
// see also https://www.botshop.co.za/how-to-use-a-ph-probe-and-sensor/
// and https://www.diymore.cc/products/diymore-liquid-ph-value-detection-detect-sensor-module-monitoring-control-for-arduino-m
// the pH sensor is a 5V part and needs either a voltage divider, or an I2C 5V ADC (this would be better as it should be more precise

void PH_Sensor::begin() {
  _ads.begin();

  _slope = -0.0010817;
  _offset = 21.6965;
}

void PH_Sensor::handle() {
  unsigned long int avgValue; 
  int buf[10];

#ifdef DEBUG_PH
  Serial.println("ph readings: ");
#endif

  for(int i = 0; i < 10; i++) { 
    buf[i] = _ads.readADC_SingleEnded(_input);
#ifdef DEBUG_PH
    Serial.println(buf[i]);
#endif
    delay(50);
  }

  for(int i = 0; i < 9; i++)  {
    for(int j = i + 1; j < 10; j++) {
      if(buf[i] > buf[j]) {
	int temp = buf[i];
	buf[i] = buf[j];
	buf[j] = temp;
      }
    }
  }

  avgValue = 0;
  for(int i = 2; i < 8; i++)
    avgValue += buf[i];

  //  float pHVol = (float)avgValue*5.0/1024/6;
  float pHVol = avgValue/6.0;

  // these values need to be calculated according to sensor calibration
  //  _ph = -5.70 * pHVol + 21.34;

#ifdef DEBUG_PH
  Serial.print("pH avgValue ");
  Serial.println(pHVol);

  Serial.print("_slope ");
  Serial.println(_slope);
  Serial.print("_offset ");
  Serial.println(_offset);

  Serial.print("_slope * phVol ");
  Serial.println(_slope*pHVol);
#endif  

  _ph = _slope * pHVol + _offset;

#ifdef DEBUG_PH
  Serial.print("computed pH ");
  Serial.println(_ph);
#endif
}
