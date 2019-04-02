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

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display;

#include "bme280_sensor.h"
BME280_Sensor bme280(MQTT_UPDATE_FREQUENCY, 0, 0, false);

#include "sonar_sensor.h"
Sonar_Sensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN,  MQTT_UPDATE_FREQUENCY, 0, 0, false);

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
// DeviceAddress sensor1 = { 0x28, 0x3F, 0x0B, 0x77, 0x91, 0x06, 0x02, 0x1C };
// DeviceAddress sensor1 = { 0x28, 0x4B, 0x8, 0x46, 0x92, 0x09, 0x02, 0x2D };
// DeviceAddress sensor1 = { 0x28, 0x3F, 0xB, 0x77, 0x91, 0x6, 0x2, 0x0 };
DeviceAddress sensor1 = { 0x28, 0xE6, 0xB9, 0x46, 0x92, 0x10, 0x2, 0xE7 };

#include "ph_sensor.h"
PH_Sensor ph_sensor(PH_ADC_INPUT, MQTT_UPDATE_FREQUENCY, 0, 0, false);

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


#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
static WiFiClient wifi_aio_client;
Adafruit_MQTT_Client mqtt(&wifi_aio_client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish temperature_feed(&mqtt, AIO_USERNAME "/feeds/waterball.temperature");
Adafruit_MQTT_Publish humidity_feed(&mqtt, AIO_USERNAME "/feeds/waterball.humidity");
Adafruit_MQTT_Publish pressure_feed(&mqtt, AIO_USERNAME "/feeds/waterball.pressure");

Adafruit_MQTT_Publish distance_feed(&mqtt, AIO_USERNAME "/feeds/waterball.distance");

Adafruit_MQTT_Publish red_feed(&mqtt, AIO_USERNAME "/feeds/waterball.red");
Adafruit_MQTT_Publish green_feed(&mqtt, AIO_USERNAME "/feeds/waterball.green");
Adafruit_MQTT_Publish blue_feed(&mqtt, AIO_USERNAME "/feeds/waterball.blue");
Adafruit_MQTT_Publish lux_feed(&mqtt, AIO_USERNAME "/feeds/waterball.lux");

Adafruit_MQTT_Publish uptime_feed(&mqtt, AIO_USERNAME "/feeds/waterball.uptime");
Adafruit_MQTT_Publish freeheap_feed(&mqtt, AIO_USERNAME "/feeds/waterball.freeheap");


#include <IFTTTWebhook.h>
IFTTTWebhook ifttt(IFTTT_API_KEY, IFTTT_EVENT_NAME);

void mqtt_connect(void) {
  int8_t ret;

 Serial.print("Connecting to Adafruit IO... ");

  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println("Wrong protocol"); break;
      case 2: Serial.println("ID rejected"); break;
      case 3: Serial.println("Server unavail"); break;
      case 4: Serial.println("Bad user/pass"); break;
      case 5: Serial.println("Not authed"); break;
      case 6: Serial.println("Failed to subscribe"); break;
      default: Serial.println("Connection failed"); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println("Retrying connection...");
    delay(5000);
  }

  Serial.println("Adafruit IO Connected!");
}

#include <rom/rtc.h>

const char* reboot_reason(int code) {
  switch(code) {
    case 1 : return "POWERON_RESET";          /**<1, Vbat power on reset*/
    case 3 : return "SW_RESET";               /**<3, Software reset digital core*/
    case 4 : return "OWDT_RESET";             /**<4, Legacy watch dog reset digital core*/
    case 5 : return "DEEPSLEEP_RESET";        /**<5, Deep Sleep reset digital core*/
    case 6 : return "SDIO_RESET";             /**<6, Reset by SLC module, reset digital core*/
    case 7 : return "TG0WDT_SYS_RESET";       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : return "TG1WDT_SYS_RESET";       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : return "RTCWDT_SYS_RESET";       /**<9, RTC Watch dog Reset digital core*/
    case 10 : return "INTRUSION_RESET";       /**<10, Instrusion tested to reset CPU*/
    case 11 : return "TGWDT_CPU_RESET";       /**<11, Time Group reset CPU*/
    case 12 : return "SW_CPU_RESET";          /**<12, Software reset CPU*/
    case 13 : return "RTCWDT_CPU_RESET";      /**<13, RTC Watch dog Reset CPU*/
    case 14 : return "EXT_CPU_RESET";         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : return "RTCWDT_BROWN_OUT_RESET";/**<15, Reset when the vdd voltage is not stable*/
    case 16 : return "RTCWDT_RTC_RESET";      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : return "NO_MEAN";
  }
}
  
void setup() {
  byte mac_address[6];

  delay(500);

  Serial.begin(115200);
  Serial.println("Hello World!");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println("Hello, world!");
  display.display();
  Serial.println("[display]");

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

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(WiFi.localIP());
  display.display();

  ifttt.trigger("reboot", reboot_reason(rtc_get_reset_reason(0)),  reboot_reason(rtc_get_reset_reason(1)));
  Serial.println("[IFTTT]");

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

  mqtt_connect();
  Serial.println("[AIO]");

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

  sonar.begin();
  Serial.println("[sonar]");

  sensors.begin();
  Serial.println("[dallas]");

  ph_sensor.begin();
  Serial.println("[ph]");

  update_mqtt.attach(MQTT_UPDATE_FREQUENCY, []() { update_mqtt_flag = true; });
  update_adafruit.attach(ADAFRUIT_UPDATE_FREQUENCY, []() { update_adafruit_flag = true; });
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
  if(! mqtt.ping(3)) {
    if(! mqtt.connected())
      mqtt_connect();
  }

  if(!mqtt_client.connected()) {
    mqtt_client.connect(MQTT_SERVER, MQTT_USERNAME, MQTT_PASSWORD);
    mqtt_client.subscribe("/hydro/rgb");
    mqtt_client.subscribe("#");
  }

  mqtt_client.loop();

  ArduinoOTA.handle();

  if(update_mqtt_flag || update_rest_flag || update_adafruit_flag) {
    bme280.handle();
    sonar.handle();
    ph_sensor.handle();

    unsigned distance = 0;
    int temperature = 0, humidity = 0, pressure = 0;
    int16_t water_temp = 0;
    float ph = 0;

    sensors.requestTemperatures();
    water_temp = sensors.getTempC(sensor1);
    Serial.printf("Dallas %d\n", water_temp);
    Serial.println(sensors.getTempC(sensor1));

    distance = sonar.distance_cm();
    Serial.printf("SONAR distance %u\n", distance);

    temperature = bme280.temperature();
    humidity = bme280.humidity();
    pressure = bme280.pressure();

    temperature_feed.publish(temperature);

    Serial.printf("Temperature %d\n", temperature);
    Serial.printf("Pressure %d\n", pressure);
    Serial.printf("Humidity %d\n", humidity);

    ph = ph_sensor.ph();

    Serial.print("pH ");
    Serial.println(ph);

    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.printf("%dcm\n", distance);
    display.printf("%dF %d%%\n", (int)(temperature*9.0/5 + 32), humidity);
    display.display();

    if(update_adafruit_flag) {
      update_adafruit_flag = false;

      distance_feed.publish(distance);
      pressure_feed.publish(pressure);
      humidity_feed.publish(humidity);
      lux_feed.publish(lux);
      red_feed.publish(red);
      green_feed.publish(green);
      blue_feed.publish(blue);

      uptime_feed.publish((unsigned)uptime.uptime()/1000);
      Serial.printf("Uptime %.2f seconds\n", uptime.uptime() / 1000.0);

      freeheap_feed.publish(ESP.getFreeHeap());
      Serial.printf("Free heap %u bytes\n", ESP.getFreeHeap());
    }

    if(update_rest_flag || update_mqtt_flag) {
      char buffer[500];
      snprintf(buffer, 500, "{\"temperature\": %d, \"humidity\": %d, \"pressure\": %d, \"red\": %d, \"green\": %d, \"blue\": %d, \"lux\": %d, \"water_temperature\": %d, \"distance\": %d, \"ph\": %f, \"freeheap\": %d, \"uptime\": %lu, \"timestamp\": %lu }",
	     temperature, humidity, pressure,
	     red, green, blue, lux,
	     water_temp,
	     distance,
	     ph,
	     ESP.getFreeHeap(), uptime.uptime()/1000,
	     time(NULL));

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
