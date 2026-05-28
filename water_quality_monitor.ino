/*
 * water_quality_monitor.ino
 * ═══════════════════════════════════════════════════════════════════════════
 * Raspberry Pi Pico — Water Quality Monitor
 * Arduino IDE (Philhower RP2040 core)
 *
 * Sensors:
 *   DS18B20   Temperature  GP22  (OneWire, digital)
 *   TDS       SEN0244      GP28 / A2
 *   pH        Analog       GP27 / A1
 *   Turbidity Analog       GP26 / A0
 *   Depth     (stub/random) GP29 / A3  ← replace Depth_Sensor.cpp when ready
 *
 * SD card (SPI):
 *   MISO GP16 | CS GP17 | SCK GP18 | MOSI GP19
 *
 * Output CSV columns:
 *   tds_raw, tds_filtered,
 *   ph_raw, ph_filtered,
 *   turbidity_raw, turbidity_filtered,
 *   temperature_raw, temperature_filtered,
 *   depth_raw, depth_filtered,
 *   timestamp_s
 *
 * Required libraries (Arduino Library Manager):
 *   OneWire            by Paul Stoffregen
 *   DallasTemperature  by Miles Burton
 *   SD                 (built-in with Philhower core)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <SPI.h>
#include <SD.h>

#include "KalmanFilter.h"
#include "DS18B20_Sensor.h"
#include "TDS_Sensor.h"
#include "PH_Sensor.h"
#include "Turbidity_Sensor.h"
#include "Depth_Sensor.h"
#include "Autoencoder.h"

// ── Configuration ──────────────────────────────────────────────────────────

#define LOG_INTERVAL_MS    10000UL   // 10 seconds between readings
#define CSV_FILENAME       "sensor_log.csv"
#define ANOMALY_FILENAME   "anomaly_log.csv"
#define SD_CS_PIN          17

// Safe ranges for alert asterisk (*) in Serial output
#define PH_MIN    6.5f
#define PH_MAX    8.5f
#define TDS_MAX   500.0f
#define TURB_MAX  200.0f   // NTU — adjust for your application
#define TEMP_MIN  0.0f
#define TEMP_MAX  35.0f

// Kalman filters (one per channel)
// KalmanFilter(measurementNoise R, processNoise Q)
// Higher R = smoother / slower.  Higher Q = reacts faster.

KalmanFilter kf_tds        (5.0f,  0.10f);
KalmanFilter kf_ph         (0.5f,  0.01f);
KalmanFilter kf_turbidity  (4.0f,  0.10f);
KalmanFilter kf_temperature(0.3f,  0.01f);
KalmanFilter kf_depth      (3.0f,  0.05f);

// Autoencoder 
// Input order: tds, ph, turbidity, temperature, depth
// Set min/max to the realistic physical range of each sensor.
// The network normalises readings to [0,1] using these bounds.

Autoencoder autoencoder;

const float AE_MIN[AE_INPUT_SIZE] = {   0.0f,  0.0f,    0.0f,  0.0f,  0.0f };
const float AE_MAX[AE_INPUT_SIZE] = { 500.0f, 14.0f, 4000.0f, 40.0f, 10.0f };

// ── Globals ────────────────────────────────────────────────────────────────

bool sdReady        = false;
unsigned long lastLogTime = 0;

// ── Helpers ────────────────────────────────────────────────────────────────

bool isOutOfRange(float v, float lo, float hi) {
    return (v < lo || v > hi);
}

void writeCSVHeader(Print& out) {
    out.println(
        "tds_raw,tds_filtered,"
        "ph_raw,ph_filtered,"
        "turbidity_raw,turbidity_filtered,"
        "temperature_raw,temperature_filtered,"
        "depth_raw,depth_filtered,"
        "timestamp_s"
    );
}

void writeAnomalyHeader(Print& out) {
    out.println("timestamp_s,mse,threshold,tds_f,ph_f,turbidity_f,temperature_f,depth_f");
}

void writeAnomalyRow(Print& out,
                     unsigned long ts, float mse, float thresh,
                     float tdsF, float phF, float turbF, float tempF, float depF)
{
    char buf[120];
    snprintf(buf, sizeof(buf),
        "%lu,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%.2f",
        ts, mse, thresh, tdsF, phF, turbF, tempF, depF);
    out.println(buf);
}

void writeCSVRow(Print& out,
                 float tdsR,   float tdsF,
                 float phR,    float phF,
                 float turbR,  float turbF,
                 float tempR,  float tempF,
                 float depR,   float depF,
                 unsigned long ts)
{
    // Build row as a char buffer to avoid many Serial.print calls
    char buf[160];
    snprintf(buf, sizeof(buf),
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu",
        tdsR,  tdsF,
        phR,   phF,
        turbR, turbF,
        tempR, tempF,
        depR,  depF,
        ts);
    out.println(buf);
}

// Pretty human-readable block for Serial monitor
void printReadable(float tdsR,  float tdsF,
                   float phR,   float phF,
                   float turbR, float turbF,
                   float tempR, float tempF,
                   float depR,  float depF)
{
    Serial.println(F("# ─────────────────────────────────────────────"));

    Serial.print(F("#  TDS         raw="));  Serial.print(tdsR,  1);
    Serial.print(F(" ppm  filtered="));      Serial.print(tdsF,  1);
    Serial.print(F(" ppm  ["));              Serial.print(tdsSensor_classify(tdsF));
    if (tdsF > TDS_MAX) Serial.print(F(" *ALERT*"));
    Serial.println(F("]"));

    Serial.print(F("#  pH          raw="));  Serial.print(phR,   2);
    Serial.print(F("     filtered="));       Serial.print(phF,   2);
    Serial.print(F("     ["));               Serial.print(phSensor_classify(phF));
    if (isOutOfRange(phF, PH_MIN, PH_MAX)) Serial.print(F(" *ALERT*"));
    Serial.println(F("]"));

    Serial.print(F("#  Turbidity   raw="));  Serial.print(turbR, 1);
    Serial.print(F(" NTU  filtered="));      Serial.print(turbF, 1);
    Serial.print(F(" NTU  ["));              Serial.print(turbiditySensor_classify(turbF));
    if (turbF > TURB_MAX) Serial.print(F(" *ALERT*"));
    Serial.println(F("]"));

    Serial.print(F("#  Temperature raw="));  Serial.print(tempR, 2);
    Serial.print(F(" °C   filtered="));      Serial.print(tempF, 2);
    Serial.print(F(" °C   ["));              Serial.print(tempSensor_classify(tempF));
    if (isOutOfRange(tempF, TEMP_MIN, TEMP_MAX)) Serial.print(F(" *ALERT*"));
    Serial.println(F("]"));

    Serial.print(F("#  Depth       raw="));  Serial.print(depR,  1);
    Serial.print(F(" cm   filtered="));      Serial.print(depF,  1);
    Serial.print(F(" cm   ["));              Serial.print(depthSensor_classify(depF));
    Serial.println(F("]"));

    Serial.println(F("# ─────────────────────────────────────────────"));
}

// ── Setup ──────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000);

    Serial.println(F("# Water Quality Monitor — booting..."));

    analogReadResolution(12);   // 12-bit ADC (0–4095) on Pico

    // Initialise each sensor module
    tempSensor_init();
    tdsSensor_init();
    phSensor_init();
    turbiditySensor_init();
    depthSensor_init();

    // Autoencoder — set normalisation bounds
    autoencoder.setNormParams(AE_MIN, AE_MAX);
    Serial.println(F("# Autoencoder ready. Training for 50 samples before anomaly detection starts."));

    // SD card
    Serial.print(F("# SD card... "));
    if (SD.begin(SD_CS_PIN)) {
        sdReady = true;
        Serial.println(F("OK"));

        if (!SD.exists(CSV_FILENAME)) {
            File f = SD.open(CSV_FILENAME, FILE_WRITE);
            if (f) { writeCSVHeader(f); f.close(); }
        } else {
            Serial.println(F("# Appending to existing file."));
        }

        if (!SD.exists(ANOMALY_FILENAME)) {
            File f = SD.open(ANOMALY_FILENAME, FILE_WRITE);
            if (f) { writeAnomalyHeader(f); f.close(); }
        }
    } else {
        Serial.println(F("FAILED — SD logging disabled."));
    }

    // Always print CSV header to Serial so the C++ dashboard can parse it
    writeCSVHeader(Serial);

    lastLogTime = millis() - LOG_INTERVAL_MS;  // trigger first reading immediately
    Serial.println(F("# Ready."));
}

// ── Main Loop ──────────────────────────────────────────────────────────────

void loop() {
    if (millis() - lastLogTime < LOG_INTERVAL_MS) return;
    lastLogTime = millis();

    // ── Read raw values ──────────────────────────────────────────────────
    // Temperature first — TDS needs it for compensation
    float tempRaw  = tempSensor_readC();
    float tdsRaw   = tdsSensor_readPPM(tempRaw > -100.0f ? tempRaw : 25.0f);
    float phRaw    = phSensor_readPH();
    float turbRaw  = turbiditySensor_readNTU();
    float depthRaw = depthSensor_readCM();

    // Apply Kalman filters
    float tdsF   = kf_tds.update(tdsRaw);
    float phF    = kf_ph.update(phRaw);
    float turbF  = kf_turbidity.update(turbRaw);
    float tempF  = kf_temperature.update(tempRaw > -100.0f ? tempRaw : tempF);
    float depthF = kf_depth.update(depthRaw);

    unsigned long timestamp = millis() / 1000UL;

    // Autoencoder train or detect 
    // Input vector order must match AE_MIN/AE_MAX: tds, ph, turbidity, temp, depth
    float aeInput[AE_INPUT_SIZE] = { tdsF, phF, turbF, tempF, depthF };

    if (!autoencoder.isReady()) {
        // Warm-up phase: train on every reading
        float mse = autoencoder.train(aeInput);
        Serial.print(F("# [AE TRAINING] sample="));
        Serial.print(autoencoder.trainCount());
        Serial.print(F("/"));
        Serial.print(AE_WARMUP_SAMPLES);
        Serial.print(F("  MSE="));
        Serial.println(mse, 5);
    } else {
        // Detection phase: check reconstruction error against dynamic threshold
        float mse    = autoencoder.reconstructionError(aeInput);
        float thresh = autoencoder.threshold();
        bool  anomaly = (mse > thresh);

        Serial.print(F("# [AE] MSE="));
        Serial.print(mse, 5);
        Serial.print(F("  threshold="));
        Serial.print(thresh, 5);

        if (anomaly) {
            Serial.println(F("  *** ANOMALY DETECTED ***"));

            // Log anomaly to SD
            if (sdReady) {
                File af = SD.open(ANOMALY_FILENAME, FILE_WRITE);
                if (af) {
                    writeAnomalyRow(af, timestamp, mse, thresh,
                                    tdsF, phF, turbF, tempF, depthF);
                    af.close();
                }
            }

            // Flash LED rapidly to signal anomaly (5 quick blinks)
            for (int b = 0; b < 5; b++) {
                digitalWrite(LED_BUILTIN, HIGH); delay(80);
                digitalWrite(LED_BUILTIN, LOW);  delay(80);
            }
        } else {
            Serial.println(F("  OK"));
        }
    }

    // ── Output CSV line to Serial ─────────────────────────────────────────
    writeCSVRow(Serial,
                tdsRaw,  tdsF,
                phRaw,   phF,
                turbRaw, turbF,
                tempRaw, tempF,
                depthRaw, depthF,
                timestamp);

    // ── Human-readable block (lines starting with # are ignored by C++ CSV reader)
    printReadable(tdsRaw, tdsF, phRaw, phF, turbRaw, turbF, tempRaw, tempF, depthRaw, depthF);

    // ── Write to SD ───────────────────────────────────────────────────────
    if (sdReady) {
        File f = SD.open(CSV_FILENAME, FILE_WRITE);
        if (f) {
            writeCSVRow(f,
                        tdsRaw,  tdsF,
                        phRaw,   phF,
                        turbRaw, turbF,
                        tempRaw, tempF,
                        depthRaw, depthF,
                        timestamp);
            f.close();
        } else {
            Serial.println(F("# SD write error"));
        }
    }

    // Blink LED to confirm a reading was taken
    digitalWrite(LED_BUILTIN, HIGH); delay(80); digitalWrite(LED_BUILTIN, LOW);
}
