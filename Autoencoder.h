#pragma once

/*
 * Autoencoder.h
 * ═══════════════════════════════════════════════════════════════════════════
 * Lightweight feedforward autoencoder for anomaly detection.
 * Runs entirely on the RP2040 — no external ML libraries.
 *
 * Architecture:
 *   Encoder:  INPUT_SIZE → HIDDEN_SIZE → BOTTLENECK_SIZE
 *   Decoder:  BOTTLENECK_SIZE → HIDDEN_SIZE → INPUT_SIZE
 *
 * For 5 sensors:
 *   5 → 3 → 2 → 3 → 5
 *
 * Training:
 *   Call train() on every "normal" reading during the warm-up period.
 *   Weights are updated via gradient descent (backpropagation).
 *
 * Detection:
 *   Call reconstruct() to get the output, then reconstructionError()
 *   to get MSE. If MSE > threshold → anomaly.
 *
 * All weights/biases are stored as float arrays — fits in ~2KB RAM.
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <math.h>

// ── Architecture constants ─────────────────────────────────────────────────
#define AE_INPUT_SIZE      5
#define AE_HIDDEN_SIZE     3
#define AE_BOTTLENECK_SIZE 2

// ── Tuning ─────────────────────────────────────────────────────────────────
#define AE_LEARNING_RATE   0.01f
#define AE_TRAIN_EPOCHS    5      // passes per new sample during training
#define AE_WARMUP_SAMPLES  50     // readings before anomaly detection starts

class Autoencoder {
public:
    Autoencoder();

    // ── Normalisation ──────────────────────────────────────────────────────
    // Call setNormParams() once you know the expected ranges of your sensors.
    // Values are normalised to [0,1] before entering the network.
    // Order: tds, ph, turbidity, temperature, depth
    void setNormParams(const float minVals[AE_INPUT_SIZE],
                       const float maxVals[AE_INPUT_SIZE]);

    // ── Training ───────────────────────────────────────────────────────────
    // Feed a raw sensor reading during warm-up.
    // Returns the reconstruction MSE after this update.
    float train(const float input[AE_INPUT_SIZE]);

    // ── Inference ──────────────────────────────────────────────────────────
    // Forward pass only — fills output[] with reconstructed values (normalised).
    void reconstruct(const float input[AE_INPUT_SIZE],
                     float output[AE_INPUT_SIZE]);

    // Mean squared error between input and reconstruction (normalised space).
    float reconstructionError(const float input[AE_INPUT_SIZE]);

    // True once enough training samples have been seen.
    bool isReady() const { return _trainCount >= AE_WARMUP_SAMPLES; }
    int  trainCount() const { return _trainCount; }

    // Dynamic threshold: mean + 3*stddev of training errors seen so far.
    float threshold() const { return _errMean + 3.0f * sqrtf(_errVar); }

private:
    // ── Weights & biases ──────────────────────────────────────────────────
    // Encoder layer 1: INPUT → HIDDEN
    float _w1[AE_HIDDEN_SIZE][AE_INPUT_SIZE];
    float _b1[AE_HIDDEN_SIZE];

    // Encoder layer 2: HIDDEN → BOTTLENECK
    float _w2[AE_BOTTLENECK_SIZE][AE_HIDDEN_SIZE];
    float _b2[AE_BOTTLENECK_SIZE];

    // Decoder layer 1: BOTTLENECK → HIDDEN
    float _w3[AE_HIDDEN_SIZE][AE_BOTTLENECK_SIZE];
    float _b3[AE_HIDDEN_SIZE];

    // Decoder layer 2: HIDDEN → OUTPUT
    float _w4[AE_INPUT_SIZE][AE_HIDDEN_SIZE];
    float _b4[AE_INPUT_SIZE];

    // ── Normalisation params ──────────────────────────────────────────────
    float _minV[AE_INPUT_SIZE];
    float _maxV[AE_INPUT_SIZE];

    // ── Training state ────────────────────────────────────────────────────
    int   _trainCount;
    float _errMean;   // running mean of reconstruction errors
    float _errVar;    // running variance (Welford's online algorithm)
    float _errM2;     // for Welford's

    // ── Internal helpers ──────────────────────────────────────────────────
    void  _normalise(const float raw[AE_INPUT_SIZE],
                           float norm[AE_INPUT_SIZE]) const;
    float _sigmoid(float x) const;
    float _sigmoidDeriv(float y) const;  // y = sigmoid(x) already computed

    // Forward pass — populates all layer activations
    void _forward(const float norm[AE_INPUT_SIZE],
                  float h1[AE_HIDDEN_SIZE],
                  float bn[AE_BOTTLENECK_SIZE],
                  float h2[AE_HIDDEN_SIZE],
                  float out[AE_INPUT_SIZE]) const;

    // Backprop + weight update
    void _backward(const float norm[AE_INPUT_SIZE],
                   const float h1[AE_HIDDEN_SIZE],
                   const float bn[AE_BOTTLENECK_SIZE],
                   const float h2[AE_HIDDEN_SIZE],
                   const float out[AE_INPUT_SIZE]);

    // Update running error statistics (Welford)
    void _updateErrorStats(float err);

    // Xavier weight initialisation
    void _initWeights();

    // Simple pseudo-random float in [-scale, scale]
    float _randWeight(float scale);
    uint32_t _rng;  // LCG state
};
