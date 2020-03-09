#include <Arduino.h>

#include "config.h"
#include "hw.h"
#include "waterball.h"

#include <multiball/app.h>
#include <multiball/wifi.h>
#include <multiball/homebus.h>
#include <multiball/uptime.h>

#ifdef USE_DIAGNOSTICS
#include <multiball/diagnostics.h>
#endif

#include "sensors/bme280.h"
#include "sensors/dallas.h"
#include "sensors/tofl.h"

static Uptime uptime;

void waterball_setup() {
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

static boolean waterball_air_update(char* buf, size_t buf_len) {
  snprintf(buf, buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.air-sensor\": { \"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f }",
	   homebus_uuid(),
	   bme280_temperature() + AIR_TEMPERATURE_ADJUSTMENT, bme280_humidity(), bme280_pressure());

  return true;
}

static boolean waterball_distance_update(char* buf, size_t buf_len) {
  snprintf(buf, buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.distance-sensor\": { \"distance\": %u } }",
	   homebus_uuid(),
	   tofl_distance());

  return true;
}

static boolean waterball_water_update(char* buf, size_t buf_len) {
  float dallas_temp;

  if(!dallas_temperature(&dallas_temp))
    return false;

  snprintf(buf, buf_len, 
	   "{ \"id\": \"%s\", \"org.homebus.experimental.water-sensor\": { \"temperature\": %.2f } }",
	   homebus_uuid(),
	   dallas_temp);

    return true;
}

static boolean waterball_system_update(char* buf, size_t buf_len) {
    static boolean posted = false;
    static IPAddress old_ip = IPAddress(0, 0, 0, 0);
    static String mac_address = WiFi.macAddress();
    IPAddress ip_addr = WiFi.localIP();

    if(ip_addr == old_ip && posted)
      return false;

  snprintf(buf,
	   buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.waterball-system\": { \"name\": \"%s\", \"build\": \"%s\", \"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" } }",
	   homebus_uuid(),
	   App.hostname().c_str(), App.build_info().c_str(), ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3], mac_address.c_str()
	   );

#ifdef VERBOSE
  Serial.println(buf);
#endif

    posted = true;
    old_ip = ip_addr;

    return true;
  }

static boolean waterball_diagnostic_update(char* buf, size_t buf_len) {
  snprintf(buf, buf_len, "{ \"id\": \"%s\", \"org.homebus.experimental.waterball-diagnostic\": { \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": %d, \"wifi_failures\": %d } }",
	   homebus_uuid(),
	   ESP.getFreeHeap(), uptime.uptime()/1000, WiFi.RSSI(), App.boot_count(), App.wifi_failures());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}


#define MAX_BUF_LEN 512

void waterball_loop() {
  char buf[MAX_BUF_LEN];

  static unsigned long next_update = 0;

  if(millis() < next_update)
    return;

  next_update = millis() + UPDATE_DELAY;

  if(waterball_air_update(buf, MAX_BUF_LEN))
    homebus_publish_to("org.homebus.experimental.air-sensor", buf);

  if(waterball_distance_update(buf, MAX_BUF_LEN))
    homebus_publish_to("org.homebus.experimental.distance-sensor", buf);

  if(waterball_water_update(buf, MAX_BUF_LEN))
    homebus_publish_to("org.homebus.experimental.water-sensor", buf);

  if(waterball_system_update(buf, MAX_BUF_LEN))
    homebus_publish_to("org.homebus.experimental.waterball-system", buf);

  if(waterball_diagnostic_update(buf, MAX_BUF_LEN))
    homebus_publish_to("org.homebus.experimental.waterball-diagnostic", buf);
}
