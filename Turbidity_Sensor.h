#pragma once

/*
 * Turbidity_Sensor.h
 * Analog turbidity sensor on GP26 / A0.
 *
 * Wiring:
 *   Sensor VCC -> 5V (or 3.3V — check your module)
 *   Sensor GND -> GND
 *   Sensor OUT -> GP26 (A0)
 *
 * NOTE: If your module runs on 5V, its analog output may exceed 3.3V.
 * In that case add a voltage divider: 10kΩ + 20kΩ to bring 5V → 3.3V.
 * Set TURBIDITY_VOLTAGE_SCALE accordingly (default 1.0 = no divider).
 */

#define TURBIDITY_PIN           A0   // GP26
#define TURBIDITY_VOLTAGE_SCALE 1.0f // Set to (R1+R2)/R2 if using divider

// Call once in setup()
void turbiditySensor_init();

// Returns turbidity in NTU (0–4000)
float turbiditySensor_readNTU();

const char* turbiditySensor_classify(float ntu);
