#include "config.h"
#include "hw.h"

#include "indicator.h"
#include "mqtt.h"

#include <WiFi.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <FastLED.h>

#define INDICATOR_INTERVAL 0.5

static Ticker indicator_ticker;

extern PubSubClient mqtt_client;

static bool indicator_flash = true;
static bool indicator_on = true;

static CRGB leds[1];

static CRGB green(0, 255, 0), red(255, 0, 0), amber(255, 194, 0), black(0, 0, 0), blue(0, 0, 255);

void indicator_loop() {
  if(WiFi.status() == WL_CONNECTED && mqtt_is_connected()) {
    leds[0] = green;
    FastLED.show();

    indicator_flash = false;
    return;
  }

  indicator_flash = true;
  if(WiFi.status() != WL_CONNECTED) {
    leds[0] = red;
    FastLED.show();

    return;
  }

  leds[0] = amber;
  FastLED.show();
}

void indicator_callback() {
  if(!indicator_flash)
    return;

  if(indicator_on) {
    indicator_on = false;
    leds[0] = black;

    FastLED.show();
    return;
  }

  indicator_on = true;
  indicator_loop();    
}

void indicator_setup() {
  FastLED.addLeds<WS2812B, LED_DATA__PIN, GRB>(leds, 1);
  FastLED.setBrightness(64);

  leds[0] = red;
  FastLED.show();
  Serial.println("red");
  delay(2000);

  leds[0] = green;
  FastLED.show();
  Serial.println("green");
  delay(2000);

  leds[0] = blue;
  FastLED.show();
  Serial.println("blue");
  delay(2000);

  indicator_ticker.attach(INDICATOR_INTERVAL, indicator_callback);
}
