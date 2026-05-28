#pragma once

/*
 * PH_Sensor.h
 * Analog pH sensor on GP27 / A1.
 *
 * Wiring:
 *   pH module VCC -> 3.3V
 *   pH module GND -> GND
 *   pH module OUT -> GP27 (A1)
 *
 * Calibration (IMPORTANT — do this before first use):
 *   1. Dip probe in pH 7.0 buffer solution
 *   2. Read voltage — set PH_NEUTRAL_VOLTAGE to that value
 *   3. Dip probe in pH 4.0 buffer solution
 *   4. Read voltage — set PH_ACID_VOLTAGE to that value
 *   Both constants are in PH_Sensor.cpp
 */

#define PH_PIN A1   // GP27

// Call once in setup()
void phSensor_init();

// Returns pH value (0–14)
float phSensor_readPH();

const char* phSensor_classify(float ph);
