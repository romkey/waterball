#pragma once

bool bme280_setup();
void bme280_loop();

float bme280_temperature(), bme280_humidity(), bme280_pressure();
