#pragma once

/*
 * TDS_Sensor.h
 * DFRobot Gravity TDS sensor (SEN0244) on GP28 / A2.
 *
 * Wiring:
 *   Sensor "+"  -> 3.3V
 *   Sensor "-"  -> GND
 *   Sensor "A"  -> GP28 (A2)
 *
 * No voltage divider needed — output stays within 0–2.3V.
 */

#define TDS_PIN A2   // GP28

// Call once in setup()
void tdsSensor_init();

// Pass current water temperature (°C) for compensation.
// Returns TDS in ppm. Needs tempC from DS18B20.
float tdsSensor_readPPM(float tempC);

const char* tdsSensor_classify(float ppm);
