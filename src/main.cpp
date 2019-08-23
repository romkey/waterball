#include <Arduino.h>

#ifdef ESP8266
#include <ESP8266mDNS.h>
#else
#include <ESPmDNS.h>
#endif

#include <SPIFFS.h>

#include "config.h"
#include "hw.h"

#include "multiball/wifi.h"
#include "multiball/ota_updates.h"
#include "multiball/mqtt.h"
#include "multiball/indicator.h"

#include "homebus_mqtt.h"

#include "sensors/bme280.h"
#include "sensors/dallas.h"
#include "sensors/tofl.h"

#ifdef BUILD_INFO

// CPP weirdness to turn a bare token into a string
#define STRINGIZE_NX(A) #A
#define STRINGIZE(A) STRINGIZE_NX(A)

char build_info[] = STRINGIZE(BUILD_INFO);
#else
char build_info[] = "not set";
#endif

// used to store persistent data across crashes/reboots
// cleared when power cycled or re-flashed
#ifdef ESP8266
int bootCount = 0;
#else
RTC_DATA_ATTR int bootCount = 0;
#endif

void setup() {
  const char* hostname = "";
  const char *wifi_credentials[] = {
    WIFI_SSID1, WIFI_PASSWORD1,
    WIFI_SSID2, WIFI_PASSWORD2,
    WIFI_SSID3, WIFI_PASSWORD3
  };

  bootCount++;

  delay(500);

  Serial.begin(115200);
  Serial.println("Hello World!");
  Serial.printf("Build %s\n", build_info);

  if(!SPIFFS.begin(true))
    Serial.println("SPIFFS Mount Failed");
  else
    Serial.println("[spiffs]");

  indicator_setup(1);
  Serial.println("[indicator]");

  if(wifi_begin(wifi_credentials, 3)) {
    Serial.println(WiFi.localIP());
    Serial.println("[wifi]");

    hostname = wifi_hostname();

    if(!MDNS.begin(hostname))
      Serial.println("Error setting up MDNS responder!");
    else
      Serial.println("[mDNS]");

  } else {
    Serial.println("wifi failure");
  }

  ota_updates_setup();
  Serial.println("[ota_updates]");

  mqtt_setup(MQTT_HOST, MQTT_PORT, MQTT_UUID, MQTT_USER, MQTT_PASS);
  Serial.println("[mqtt]");

  homebus_mqtt_setup();
  Serial.println("[homebus-mqtt]");

  if(bme280_setup())
    Serial.println("[bme280]");
  else
    Serial.println("BME280 not found");

  tofl_setup();
  Serial.println("[tofl]");

  dallas_setup();
  Serial.println("[dallas]");

#ifdef USE_PH
  ph_sensor.begin();
  Serial.println("[ph]");
#endif
}

bool status_changed = true;

void loop() {
  ota_updates_handle();

  bme280_loop();
  tofl_loop();
  dallas_loop();

  mqtt_handle();
  homebus_mqtt_loop();
}
