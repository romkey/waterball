#include <Arduino.h>

#include <SPIFFS.h>

#include "config.h"
#include "hw.h"

#include "waterball.h"

#include "multiball/app.h"
#include "multiball/wifi.h"
#include "multiball/ota_updates.h"
#include "multiball/mqtt.h"
#include "multiball/homebus.h"

MultiballApp App;

void setup() {
  const wifi_credential_t wifi_credentials[] = {
    { WIFI_SSID1, WIFI_PASSWORD1 },
    { WIFI_SSID2, WIFI_PASSWORD2 },
    { WIFI_SSID3, WIFI_PASSWORD3 }
  };

  delay(500);

  App.wifi_credentials(3, wifi_credentials);
  App.begin("waterball");
  Serial.println("[app]");

#ifdef MQTT_HOST
  Serial.println("[credentials]");
  homebus_stuff(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_UUID);
#endif

  Serial.println("[homebus]");
  homebus_configure("Waterball", "CTRLH", "Homebus", "v2");
  homebus_setup();

  Serial.println("[homebus mqtt]");
  homebus_mqtt_setup();

  waterball_setup();
  Serial.println("[waterball]");

#ifdef USE_DIAGNOSTICS
  diagnostics_setup();
  Serial.println("[diagnostics]");
#endif

  delay(500);
}

bool status_changed = true;

void loop() {
  App.handle();

  waterball_loop();
}
