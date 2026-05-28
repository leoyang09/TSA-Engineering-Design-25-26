/*
 * Depth_Sensor.cpp
 * Fake/stub depth sensor.
 * Outputs a random float between 0.0 and 10.0 cm (= 0.0 – 0.1 m).
 *
 * To replace with a real sensor later:
 *   1. Add your pin #define and any library #include at the top
 *   2. Put initialisation code inside depthSensor_init()
 *   3. Replace the random() line in depthSensor_readCM() with a real read
 */

#include "Depth_Sensor.h"
#include <Arduino.h>

void depthSensor_init() {
    // Seed the random number generator from an unconnected analog pin
    // so values differ each boot
    randomSeed(analogRead(A3));
}

float depthSensor_readCM() {
    // random(0, 101) gives an integer 0–100
    // Dividing by 10.0 gives 0.0–10.0 cm  (i.e. 0.00–0.10 m)
    return random(0, 101) / 10.0f;
}

const char* depthSensor_classify(float cm) {
    if (cm <  2.0f) return "Very Shallow";
    if (cm <  5.0f) return "Shallow";
    if (cm < 10.0f) return "Moderate";
    return                 "Deep";
}
