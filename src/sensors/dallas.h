#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>

bool dallas_setup();
void dallas_loop();

bool dallas_temperature(float*);

