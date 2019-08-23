#pragma once

#include <Arduino.h>

#include "config.h"

#ifdef USE_MQTT

#include <PubSubClient.h>

bool mqtt_connect(PubSubClient*);
void mqtt_setup();
void mqtt_loop();
void mqtt_callback(const char*, const byte*, unsigned);
void mqtt_publish(const char* topic, const char* payload, bool retain = false);
bool mqtt_is_connected();
#endif
