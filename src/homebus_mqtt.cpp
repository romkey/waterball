#include "config.h"
#include "hw.h"

#include "multiball/wifi.h"
#include "multiball/mqtt.h"
#include "homebus_mqtt.h"

#include "multiball/uptime.h"

#include "sensors/bme280.h"
#include "sensors/dallas.h"
#include "sensors/tofl.h"

static Uptime uptime;

extern RTC_DATA_ATTR int bootCount;
extern RTC_DATA_ATTR int wifi_failures;

extern bool status_changed;

static String homebus_endpoint;

static void homebus_mqtt_publish_status();

void homebus_mqtt_setup() {
  homebus_endpoint = String("/homebus/device/") + MQTT_UUID;
}

void homebus_mqtt_loop() {
  bool should_publish = false;

  static unsigned long next_update = 0;

  if(millis() > next_update) {
    should_publish = true;
    next_update = millis() + UPDATE_DELAY;
  }

  if(status_changed) {
    should_publish = true;
    status_changed = false;
  }

  if(should_publish)
    homebus_mqtt_publish_status();
}

#define MAX_STATUS_LENGTH 512

static void homebus_mqtt_publish_status() {
  char buf[MAX_STATUS_LENGTH+1];
  IPAddress local = WiFi.localIP();

  snprintf(buf, MAX_STATUS_LENGTH, "{ \"id\": \"%s\", ", MQTT_UUID);
  snprintf(buf + strlen(buf),
	   MAX_STATUS_LENGTH - strlen(buf),
	   " \"system\": {  \"name\": \"%s\", \"build\": \"%s\", \"freeheap\": %d, \"uptime\": %lu, \"ip\": \"%d.%d.%d.%d\", \"rssi\": %d, \"reboots\": %d, \"wifi_failures\": %d }, ",
	   wifi_hostname(), "", ESP.getFreeHeap(), uptime.uptime()/1000, local[0], local[1], local[2], local[3], WiFi.RSSI(), bootCount, wifi_failures);

  snprintf(buf + strlen(buf),
	   MAX_STATUS_LENGTH - strlen(buf),
	   "\"environment\": { \"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f }, ",
	   bme280_temperature() + AIR_TEMPERATURE_ADJUSTMENT, bme280_humidity(), bme280_pressure());

  snprintf(buf + strlen(buf),
	   MAX_STATUS_LENGTH - strlen(buf),
	   "\"water\": { \"ph\": null, \"distance\": %u, ",
	   tofl_distance());

  float dallas_temp;
  if(dallas_temperature(&dallas_temp))
    snprintf(buf + strlen(buf),
	     MAX_STATUS_LENGTH - strlen(buf),
	     "\"temperature\": %.2f }",
	     dallas_temp);
  else
    snprintf(buf + strlen(buf),
	     MAX_STATUS_LENGTH - strlen(buf),
	     "\"temperature\": null }");

  snprintf(buf + strlen(buf),
	   MAX_STATUS_LENGTH - strlen(buf),
	   " }");

  Serial.println(buf);
  mqtt_publish(homebus_endpoint.c_str(), buf, true);
}

// standard homebus vocabulary:
// - ping
// - restart
// waterball homebus vocabulary:
// - scan_dallas
void homebus_mqtt_callback(char const* topic, char const* msg) {
}
