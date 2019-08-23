#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <Esp.h>
#include <WiFi.h>
#endif

#include "config.h"

#include "wifi_local.h"
#include "mqtt.h"

static WiFiClient wifi_mqtt_client;
static PubSubClient mqtt_client(wifi_mqtt_client);

bool mqtt_connect() {
  if(mqtt_client.connected())
    return false;

  mqtt_client.connect(MQTT_UUID, MQTT_USER, MQTT_PASS);
  mqtt_client.subscribe(MQTT_CMD_TOPIC);

  mqtt_client.publish("/status", "\"hello world\"");

  return true;
}

void mqtt_setup() {
  mqtt_client.setServer(MQTT_HOST, MQTT_PORT);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_connect();
}

void mqtt_loop() {
  static unsigned long last_mqtt_check = 0;

  mqtt_client.loop();

  if(millis() > last_mqtt_check + 5000) {
    if(mqtt_connect())
      Serial.println("mqtt reconnect");

    last_mqtt_check = millis();
  }
}

void mqtt_publish(const char* topic, const char* payload, bool retain) {
  mqtt_client.publish(topic, payload, retain);
}

void mqtt_callback(const char* topic, const byte* payload, unsigned int length) {
  char command_buffer[length + 1];
  char* command = command_buffer;

  memcpy(command_buffer, payload, length);
  command[length] = '\0';

  // command is meant to be a valid json string, so get rid of the quotes
  if(command[0] == '"' && command[length-1] == '"') {
    command[length-1] = '\0';
    command += 1;
  }

}

bool mqtt_is_connected() {
  return mqtt_client.connected();
}
