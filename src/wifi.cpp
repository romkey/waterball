#include "config.h"

#include "wifi_local.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#else
#include <WiFi.h>
#include <WiFiMulti.h>
#endif


static char hostname[sizeof("waterball-%02x%02x%02x") + 1];

#ifdef ESP8266
static ESP8266WiFiMulti wifiMulti;
#else
static WiFiMulti wifiMulti;
#endif

#ifdef ESP8266
int wifi_failures = 0;
#else
RTC_DATA_ATTR int wifi_failures = 0;
#endif

bool wifi_setup() {
  byte mac_address[6];

  WiFi.macAddress(mac_address);
  snprintf(hostname, sizeof(hostname), "waterball-%02x%02x%02x", (int)mac_address[3], (int)mac_address[4], (int)mac_address[5]);
  Serial.printf("Hostname: %s\n", hostname);
  Serial.printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac_address[0],
		mac_address[1],
		mac_address[2],
		mac_address[3],
		mac_address[4],
		mac_address[5]);
#ifdef ESP8266
  WiFi.hostname(hostname);
#else
  WiFi.setHostname(hostname);
#endif

  wifiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
  wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);
  wifiMulti.addAP(WIFI_SSID3, WIFI_PASSWORD3);

  static int wifi_tries = 0;
  while(wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);

    if(wifi_tries++ > 100) {
      wifi_failures++;
      ESP.restart();
    }
  }
}

const char* wifi_hostname() {
  return hostname;
}

void wifi_loop() {
  if(WiFi.status() != WL_CONNECTED)
    wifiMulti.run();
}
