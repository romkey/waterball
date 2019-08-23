#include <Arduino.h>

#include <Esp.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <Ticker.h>

#include "config.h"
#include "hw.h"

#include <ArduinoOTA.h>

WiFiMulti wifiMulti;

#include "bme280_sensor.h"
BME280_Sensor bme280(MQTT_UPDATE_FREQUENCY, 0, 0, false);

#if USE_SONAR
#include "sonar_sensor.h"
Sonar_Sensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN,  MQTT_UPDATE_FREQUENCY, 0, 0, false);
#endif


#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
// DeviceAddress sensor1 = { 0x28, 0x3F, 0x0B, 0x77, 0x91, 0x06, 0x02, 0x1C };
// DeviceAddress sensor1 = { 0x28, 0x4B, 0x8, 0x46, 0x92, 0x09, 0x02, 0x2D };
// DeviceAddress sensor1 = { 0x28, 0x3F, 0xB, 0x77, 0x91, 0x6, 0x2, 0x0 };
// DeviceAddress sensor1 = { 0x28, 0xE6, 0xB9, 0x46, 0x92, 0x10, 0x2, 0xE7 };
DeviceAddress sensor1 = { 0x28, 0x28, 0xC8, 0x4D, 0x97, 0x13, 0x03, 0x6A };

#ifdef USE_PH
#include "ph_sensor.h"
PH_Sensor ph_sensor(PH_ADC_INPUT, MQTT_UPDATE_FREQUENCY, 0, 0, false);
#endif

#ifdef USE_TOFL
#include "tofl_sensor.h"
TOFL_Sensor tol_sensor(MQTT_UPDATE_FREQUENCY, 0, 0, false);
#endif

#include "uptime.h"
Uptime uptime;

static Ticker update_mqtt;
static Ticker update_rest;
static Ticker update_adafruit;
static Ticker update_heartbeat;

static bool update_mqtt_flag = true;
static bool update_rest_flag = true;
static bool update_adafruit_flag = true;
static bool update_heartbeat_flag = true;

static char hostname[sizeof(WATERBALL_HOSTNAME) + 9];


#include <PubSubClient.h>
static WiFiClient wifi_mqtt_client;
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void wifi_blink();

static PubSubClient mqtt_client(MQTT_SERVER, MQTT_PORT, mqtt_callback, wifi_mqtt_client);

  
void setup() {
  byte mac_address[6];

  delay(500);

  Serial.begin(115200);
  Serial.println("Hello World!");

  wifiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
  wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);
  wifiMulti.addAP(WIFI_SSID3, WIFI_PASSWORD3);

  WiFi.macAddress(mac_address);
  snprintf(hostname, sizeof(hostname), "%s-%02x%02x%02x", WATERBALL_HOSTNAME, (int)mac_address[3], (int)mac_address[4], (int)mac_address[5]);
  Serial.printf("Hostname is %s\n", hostname);

  WiFi.setHostname(hostname);
  while(wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("[wifi]");

  if(!MDNS.begin(hostname))
    Serial.println("Error setting up MDNS responder!");

  Serial.println("[mDNS]");

  mqtt_client.connect(MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);
  while(!mqtt_client.connected()) {
    Serial.print('+');
    delay(200);
  }

  mqtt_client.setCallback(mqtt_callback);

  if(!mqtt_client.subscribe("/hydro/rgb")) {
    Serial.println("subscribe to /hydro/rgb failed");
  }

  if(!mqtt_client.subscribe("#")) {
    Serial.println("subscribe to # failed");
  }
  Serial.println("[MQTT]");

  configTime(TZ_OFFSET, DST_OFFSET, "pool.ntp.org", "time.nist.gov");
  Serial.println("[NTP]");

#ifdef ESP32
   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      ESP.restart();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
#endif

  ArduinoOTA.begin();
  Serial.println("[ota]");

  bme280.begin();
  Serial.println("[bme280]");

#ifdef USE_SONAR
  sonar.begin();
  Serial.println("[sonar]");
#endif

#ifdef USE_TOFL
  tofl_sensor.begin();
  Serial.println("[tofl]");
#endif

  sensors.begin();
  Serial.println("[dallas]");

#ifdef USE_PH
  ph_sensor.begin();
  Serial.println("[ph]");
#endif

  update_mqtt.attach(MQTT_UPDATE_FREQUENCY, []() { update_mqtt_flag = true; });
  update_heartbeat.attach(HEARTBEAT_UPDATE_FREQUENCY, []() { update_heartbeat_flag = true; });
  update_rest.attach(REST_UPDATE_FREQUENCY, []() { update_rest_flag = true; });

  delay(500);
}

static uint16_t red = -1, green = -1, blue = -1;
static int lux = -1;
static unsigned long last_rgb_update_time = 0;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.println(">>>> MQTT_CALLBACK <<<<");
  Serial.printf("topic %s, length %d\n", topic, length);

  if(strcmp(topic, MQTT_RGB_TOPIC) != 0) {
    Serial.printf("mqtt_callback: bad topic: %s\n", topic);
    return;
  }

  char buf[length + 1];
  memcpy(buf, payload, length);
  buf[length] = '\0';

  Serial.printf(">>> MQTT payload %s\n", buf);

  // expect a message in the format r g b lux unixtime
  sscanf(buf, "%hd %hd %hd %d %lu", &red, &green, &blue, &lux, &last_rgb_update_time);
  Serial.printf("red: %d, green: %d, blue: %d, lux: %d, update time: %lu\n",
		red, green, blue, lux, last_rgb_update_time);
}

void loop() {
  if(!mqtt_client.connected()) {
    mqtt_client.connect(MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);
    mqtt_client.subscribe("/hydro/rgb");
    mqtt_client.subscribe("#");
  }

  mqtt_client.loop();

  ArduinoOTA.handle();

  if(update_mqtt_flag || update_rest_flag || update_adafruit_flag) {
    bme280.handle();

#ifdef USE_SONAR
    sonar.handle();
    unsigned distance = 0;

    distance = sonar.distance_cm();
    Serial.printf("SONAR distance %u\n", distance);
#endif

#ifdef USE_PH
    ph_sensor.handle();
    float ph = 0;

    ph = ph_sensor.ph();

    Serial.print("pH ");
    Serial.println(ph);
#endif

    int temperature = 0, humidity = 0, pressure = 0;
    int16_t water_temp = 0;

    sensors.requestTemperatures();
    water_temp = sensors.getTempC(sensor1);
    Serial.printf("Dallas %d\n", water_temp);
    Serial.println(sensors.getTempC(sensor1));

    temperature = bme280.temperature();
    humidity = bme280.humidity();
    pressure = bme280.pressure();

    Serial.printf("Temperature %d\n", temperature);
    Serial.printf("Pressure %d\n", pressure);
    Serial.printf("Humidity %d\n", humidity);

    char buffer[500];
    if(update_rest_flag || update_mqtt_flag) {
      char sm_buffer[100];

      snprintf(buffer, 500, "{\"freeheap\": %d, \"uptime\": %lu, \"timestamp\": %lu, ", ESP.getFreeHeap(), uptime.uptime()/1000, time(NULL));

      snprintf(sm_buffer, 100, "\"temperature\": %d, \"humidity\": %d, \"pressure\": %d, ", temperature, humidity, pressure);

      if(RGB_VALID)
	snprintf(sm_buffer, 100, "\"red\": %d, \"green\": %d, \"blue\": %d, \"lux\": %d, ", red, green, blue, lux);
      else
	snprintf(sm_buffer, 100, "\"red\": null, \"green\": null, \"blue\": null, \"lux\": null, ");
      strncat(buffer, sm_buffer, 500);



      if(water_temp > 0 && water_temp < 50)
	snprintf(sm_buffer, 100, "\"water_temperature\": %d, ", water_temp);
      else
	snprintf(sm_buffer, 100, "\"water_temperature\": null, ");
      strncat(buffer, sm_buffer, 500);



#ifdef USE_SONAR
      if(sonar.valid())
	snprintf(sm_buffer, 100, "\"distance\": %d, ", distance);
      else
	snprintf(sm_buffer, 100, "\"distance\": null, ");
      strncat(buffer, sm_buffer, 500);
#endif

#ifdef USE_PH
      if(ph_sensor.valid())
	snprintf(sm_buffer, 100, "\"ph\": %f ", ph);
      else
	snprintf(sm_buffer, 100, "\"ph\": null ");
      strncat(buffer, sm_buffer, 500);
#endif

      strncat(buffer, "}", 500);
    }

    Serial.println(buffer);

    if(update_mqtt_flag) {
      update_mqtt_flag = false;
      mqtt_client.publish("/aqua", buffer, true);
    }

#ifdef REST_API_ENDPOINT
    if(update_rest_flag) {
      update_rest_flag = false;

      void post(char*);
      post(buffer);
    }
#endif
  }

  if(update_heartbeat_flag) {
    update_heartbeat_flag = false;

#define HEARTBEAT_BUFFER_SIZE 256
    char buf[HEARTBEAT_BUFFER_SIZE];
    IPAddress local_ip = WiFi.localIP();
    byte mac_address[6];

    WiFi.macAddress(mac_address);

    snprintf(buf, HEARTBEAT_BUFFER_SIZE, "{ \"hostname\": \"%s\", \"ip\": \"%d.%d.%d.%d\", \"mac_address\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"freeheap\": %d, \"uptime\": %lu, \"timestamp\": %lu }",
	     hostname,
	     local_ip[0], local_ip[1], local_ip[2], local_ip[3],
	     mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5],
	     ESP.getFreeHeap(), uptime.uptime()/1000,
	     time(NULL));

    Serial.printf("heartbeat %s\n", buf);
    mqtt_client.publish("/heartbeat", buf);
  }
}

#ifdef REST_API_ENDPOINT

static const char *cert = 
"-----BEGIN CERTIFICATE-----\n" \
"MIIEnDCCA4SgAwIBAgIQYnFSTCR8WCqzmau4LvmKqzANBgkqhkiG9w0BAQsFADBU" \
"MQswCQYDVQQGEwJVUzEeMBwGA1UEChMVR29vZ2xlIFRydXN0IFNlcnZpY2VzMSUw" \
"IwYDVQQDExxHb29nbGUgSW50ZXJuZXQgQXV0aG9yaXR5IEczMB4XDTE5MDMwMTIy" \
"MDM1MloXDTE5MDgzMDAwMDAwMFowaDELMAkGA1UEBhMCVVMxEzARBgNVBAgMCkNh" \
"bGlmb3JuaWExFjAUBgNVBAcMDU1vdW50YWluIFZpZXcxEzARBgNVBAoMCkdvb2ds" \
"ZSBMTEMxFzAVBgNVBAMMDmZpcmViYXNlaW8uY29tMIIBIjANBgkqhkiG9w0BAQEF" \
"AAOCAQ8AMIIBCgKCAQEApVkDnfzsuQ2U9lCQXgSp3Cai/JgmIjgqulzTBfXaT+xj" \
"1bQGFCx35T/XxgsPwVYfVsxGg9EpahUgvf08d1YoEoY+PabqkcCXXn5Zv0vGrNNs" \
"2yIWKKIMzzHt4Fz1InFFDOO3cpliC38EyKjXBC3F7X94QDUx5rIm0zIE1hKBvEjK" \
"QPLcTFtwH/8YQiu+s4w8xx8uc9feaZI7Fk+ErN/KcIVTPhTevGz20UA75oWWiGmr" \
"gFx68x4lezoRW0fivMiMZm9dX5mcBC5Gt+i/HEW0HGgcn5h2Q7ew0RLH4Xs7CsLo" \
"3xRhjuZJLpQZfJCJVfk6Mm3AJJnl5XFcO3lvwwEYvwIDAQABo4IBVDCCAVAwEwYD" \
"VR0lBAwwCgYIKwYBBQUHAwEwKwYDVR0RBCQwIoIOZmlyZWJhc2Vpby5jb22CECou" \
"ZmlyZWJhc2Vpby5jb20waAYIKwYBBQUHAQEEXDBaMC0GCCsGAQUFBzAChiFodHRw" \
"Oi8vcGtpLmdvb2cvZ3NyMi9HVFNHSUFHMy5jcnQwKQYIKwYBBQUHMAGGHWh0dHA6" \
"Ly9vY3NwLnBraS5nb29nL0dUU0dJQUczMB0GA1UdDgQWBBQo3KJQxX+HruhUE5kb" \
"7fwA4v5N+DAMBgNVHRMBAf8EAjAAMB8GA1UdIwQYMBaAFHfCuFCaZ3Z2sS3ChtCD" \
"oH6mfrpLMCEGA1UdIAQaMBgwDAYKKwYBBAHWeQIFAzAIBgZngQwBAgIwMQYDVR0f" \
"BCowKDAmoCSgIoYgaHR0cDovL2NybC5wa2kuZ29vZy9HVFNHSUFHMy5jcmwwDQYJ" \
"KoZIhvcNAQELBQADggEBALxT+yKjqaQj38hYspjpJMdrUQ87TMUpHRss7H3g87mR" \
"bgRdyoRW9qB6+KbYio9NkMfbeIZy1gPZfbze2A8VbFQSa6k11PGge0n4621gE5Ss" \
"yBjquFn15tIc6e74X6Eq0d8jURZJE2hhtfKxm951RK9j4fgDChM3h8eImy4Ti3wl" \
"MOdW9gVkx7pgWibRi//umMLHUEkuqGd3jeke3l1h95QrS8RDhpbrF7PgMn66w9L5" \
"qjBHPmnQHzTOHa1SJlduCfQBnyuyXg77zkkzA4WekQBlXHoTIJ5e44v30FREwwsa" \
"a9CDdzFrAnUyHOTdTaFBKVWhe3m01XKZcTUEK2GXRK0=" \
"-----END CERTIFICATE-----\n";


void post(char *json) {
  HTTPClient https;

  Serial.println("posting...");
  int ret_code;
  ret_code = https.begin(REST_API_ENDPOINT, cert);
  if(ret_code) {
    int response = https.POST(json);
    if(response > 0)
      Serial.printf("HTTP status code %d\n", response);
    else
      Serial.printf("HTTPClient error %d\n", response);
  } else
    Serial.printf("https.begin failed %d\n", ret_code);

  https.end();

#if 0  
  HTTPClient http;

  http.begin(String(REST_API_ENDPOINT));
  http.addHeader("Content-Type", "application/json");
  int response = http.POST(json);
  if(response > 0) {
    Serial.printf("HTTP status code %d\n", response);
  } else {
    Serial.printf("HTTPClient error %d\n", response);
  }

  http.end();
#endif
}
#endif
