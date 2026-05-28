#pragma once

/*
 * Depth_Sensor.h
 * Stub depth sensor — returns a random depth between 0.0 and 0.1 metres.
 * Replace the body of depthSensor_readCM() with your real sensor logic later.
 *
 * Output is in CENTIMETRES to match the rest of the pipeline (0–10 cm).
 */

// Call once in setup()
void depthSensor_init();

// Returns depth in cm (0.0 – 10.0 for the stub)
float depthSensor_readCM();

const char* depthSensor_classify(float cm);
