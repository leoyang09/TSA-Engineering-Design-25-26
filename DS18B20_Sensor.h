#pragma once

/*
 * DS18B20_Sensor.h
 * One-Wire temperature sensor on GP22.
 *
 * Wiring:
 *   DS18B20 GND  -> Pico GND
 *   DS18B20 DATA -> GP22  +  4.7kΩ pullup to 3.3V   ← REQUIRED
 *   DS18B20 VCC  -> 3.3V
 *
 * Libraries required (Arduino Library Manager):
 *   "OneWire"           by Paul Stoffregen
 *   "DallasTemperature" by Miles Burton
 */

#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_PIN 22

// Call once in setup()
void tempSensor_init();

// Returns temperature in °C, or -127.0 on error
float tempSensor_readC();

// Human-readable label
const char* tempSensor_classify(float tempC);
