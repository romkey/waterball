#include <Arduino.h>

#include <ESP.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include "config.h"
#include "hw.h"

#include <ArduinoOTA.h>

WiFiMulti wifiMulti;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display;

#include "bme280_sensor.h"
BME280_Sensor bme280(UPDATE_DELAY, 0, 0, false);

#include "sonar_sensor.h"
Sonar_Sensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN,  UPDATE_DELAY, 0, 0, false);

// ---- https://randomnerdtutorials.com/esp32-with-multiple-ds18b20-temperature-sensors/
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS_PIN 0

OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1 = { 0x28, 0xFF, 0x77, 0x62, 0x40, 0x17, 0x4, 0x31 };

#include "tcs34725_sensor.h"
TCS34725_Sensor tcs34725(UPDATE_DELAY, 0, 0, false);;

#include "uptime.h"
Uptime uptime;

#include "relay.h"
Relay relay(RELAY_PIN);

#include <PubSubClient.h>
static WiFiClient wifi_mqtt_client;
void mqtt_callback(char* topic, byte* payload, unsigned int length) { };
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


// HomeBus is not ready for Prime Time so leave this out for now
#ifdef HOMEBUS

#include <HomeBus.h>
#include <HomeBusDevice.h>

HomeBus hb("Furball One",
	   "an esp8266 far, far away",
	   "HomeBus Central",
	   "0.0.1",
	   WiFi.mac_address,
	   "0-0-0-0");

HomeBusDevice hb_tsl2561_lux(&hb,
			     "Light sensor",
			     "HomeBus One",
			     100,
			     100,
			     1000,
			     true,
			     "lux",
			     "",
			     ""
			     );

HomeBusDevice hb_tsl2561_ir(&hb,
			    "Infrared sensor",
			    "HomeBus One",
			    100,
			    100,
			    1000,
			    true,
			    "ir",
			    "",
			    ""
			    );


HomeBusDevice hb_ccs811_voc(&hb,
			    "VOC",
			    "HomeBus One",
			    100,
			    100,
			    1000,
			    true,
			    "voc",
			    "",
			    ""
			    );

HomeBusDevice hb_ccs811_voc(&hb,
			    "CO2",
			    "HomeBus One",
			    100,
			    100,
			    1000,
			    false,
			    "co2",
			    "",
			    ""
			    );

#endif




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
  char hostname[sizeof(WATERBALL_HOSTNAME) + 8];
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

  //  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  wifiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
  wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);
  wifiMulti.addAP(WIFI_SSID3, WIFI_PASSWORD3);

  WiFi.macAddress(mac_address);
  snprintf(hostname, sizeof(hostname), "%s %02x%02x%02x", WATERBALL_HOSTNAME, (int)mac_address[3], (int)mac_address[4], (int)mac_address[5]);
  Serial.printf("Hostname is %s\n", hostname);

  WiFi.setHostname(hostname);

  //  while(!WiFi.isConnected()) {
  while(wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println("[wifi]");

  ifttt.trigger("reboot", reboot_reason(rtc_get_reset_reason(0)),  reboot_reason(rtc_get_reset_reason(1)));
  Serial.println("[IFTTT]");

  if(!MDNS.begin(hostname))
    Serial.println("Error setting up MDNS responder!");
  else
    Serial.println("mDNS responder started");
  Serial.println("[mDNS]");

  mqtt_connect();
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

  display.clearDisplay();
  display.setCursor(0,0);
  display.println(hostname);
  display.println(WiFi.localIP());
  display.display();

  bme280.begin();
  Serial.println("[bme280]");

  tcs34725.begin();
  Serial.println("[tcs34725]");

  sonar.begin();
  Serial.println("[sonar]]");

  sensors.begin();
  Serial.println("[dallas]");

  mqtt_connect();

  delay(500);
}


void loop() {
  if(! mqtt.ping(3)) {
    if(! mqtt.connected())
      mqtt_connect();
  }

  static unsigned long last_loop = 0;

  if(millis() - last_loop < UPDATE_DELAY)
    return;

  Serial.println("next: handles");

  ArduinoOTA.handle();

  bme280.handle();
  tcs34725.handle();
  sonar.handle();

  Serial.println("next: sonar");

  unsigned distance = 0;
  uint16_t red = 0, green = 0, blue = 0;
  int temperature = 0, humidity = 0, pressure = 0;
  int lux = 0;

  if(sonar.ready_for_update()) {
    distance = sonar.distance_cm();
    Serial.printf("SONAR distance %u\n", distance);
    distance_feed.publish(distance);
  }

  Serial.println("next: bme");

  if(bme280.ready_for_update()) {
    temperature = bme280.temperature();
    humidity = bme280.humidity();
    pressure = bme280.pressure();

    temperature_feed.publish(temperature);
    pressure_feed.publish(pressure);
    humidity_feed.publish(humidity);

    Serial.printf("Temperature %d\n", temperature);
    Serial.printf("Pressure %d\n", pressure);
    Serial.printf("Humidity %d\n", humidity);
  }

  Serial.println("next: tcs");

  if(tcs34725.ready_for_update()) {
    red = tcs34725.red();
    green = tcs34725.green();
    blue = tcs34725.blue();
    lux = tcs34725.lux();

    red_feed.publish(red);
    green_feed.publish(green);
    blue_feed.publish(blue);
    lux_feed.publish(lux);

    if(lux < 500) {
      Serial.println("relay on");
      relay.on();
    } else {
      Serial.println("relay off");
      relay.off();
    }

    Serial.printf("lux %d, red %d, green %d, blue %d\n", lux, red, green, blue);
  }

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.printf("%dcm %dl\n", distance, lux);
  display.printf("%dF %d%%\n", (int)(temperature*9.0/5 + 32), humidity);
  display.display();


  last_loop = millis();

  uptime_feed.publish((unsigned)uptime.uptime()/1000);
  Serial.printf("Uptime %.2f seconds\n", uptime.uptime() / 1000.0);

  freeheap_feed.publish(ESP.getFreeHeap());
  Serial.printf("Free heap %u bytes\n", ESP.getFreeHeap());

  char buffer[500];
  snprintf(buffer, 500, "{\"temperature\": %d, \"humidity\": %d, \"pressure\": %d, \"red\": %d, \"green\": %d, \"blue\": %d, \"lux\": %d, \"distance\": %d, \"freeheap\": %d, \"uptime\": %lu, \"timestamp\": %lu }",
	   temperature, humidity, pressure,
	   red, green, blue, lux,
	   distance,
	   ESP.getFreeHeap(), uptime.uptime()/1000,
	   time(NULL));

    Serial.println(buffer);

#ifdef REST_API_ENDPOINT
    post(buffer);
#endif
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
