/*
 * Autoencoder.cpp
 * Full implementation — backpropagation, normalisation, dynamic threshold.
 */

#include "Autoencoder.h"

// ── Constructor ────────────────────────────────────────────────────────────

Autoencoder::Autoencoder()
    : _trainCount(0)
    , _errMean(0.0f)
    , _errVar(0.0f)
    , _errM2(0.0f)
    , _rng(12345UL)
{
    // Default normalisation: 0–1 pass-through (override with setNormParams)
    for (int i = 0; i < AE_INPUT_SIZE; i++) {
        _minV[i] = 0.0f;
        _maxV[i] = 1.0f;
    }
    _initWeights();
}

// ── Public API ─────────────────────────────────────────────────────────────

void Autoencoder::setNormParams(const float minVals[AE_INPUT_SIZE],
                                const float maxVals[AE_INPUT_SIZE]) {
    for (int i = 0; i < AE_INPUT_SIZE; i++) {
        _minV[i] = minVals[i];
        _maxV[i] = maxVals[i];
    }
}

float Autoencoder::train(const float input[AE_INPUT_SIZE]) {
    float norm[AE_INPUT_SIZE];
    _normalise(input, norm);

    for (int epoch = 0; epoch < AE_TRAIN_EPOCHS; epoch++) {
        float h1[AE_HIDDEN_SIZE];
        float bn[AE_BOTTLENECK_SIZE];
        float h2[AE_HIDDEN_SIZE];
        float out[AE_INPUT_SIZE];

        _forward(norm, h1, bn, h2, out);
        _backward(norm, h1, bn, h2, out);
    }

    // Compute final MSE after update
    float mse = reconstructionError(input);
    _updateErrorStats(mse);
    _trainCount++;
    return mse;
}

void Autoencoder::reconstruct(const float input[AE_INPUT_SIZE],
                               float output[AE_INPUT_SIZE]) {
    float norm[AE_INPUT_SIZE];
    _normalise(input, norm);

    float h1[AE_HIDDEN_SIZE];
    float bn[AE_BOTTLENECK_SIZE];
    float h2[AE_HIDDEN_SIZE];
    _forward(norm, h1, bn, h2, output);
}

float Autoencoder::reconstructionError(const float input[AE_INPUT_SIZE]) {
    float out[AE_INPUT_SIZE];
    reconstruct(input, out);

    float norm[AE_INPUT_SIZE];
    _normalise(input, norm);

    float mse = 0.0f;
    for (int i = 0; i < AE_INPUT_SIZE; i++) {
        float diff = norm[i] - out[i];
        mse += diff * diff;
    }
    return mse / AE_INPUT_SIZE;
}

// Forward pass
void Autoencoder::_forward(const float norm[AE_INPUT_SIZE],
                            float h1[AE_HIDDEN_SIZE],
                            float bn[AE_BOTTLENECK_SIZE],
                            float h2[AE_HIDDEN_SIZE],
                            float out[AE_INPUT_SIZE]) const {
    // Encoder layer1 input to hidden
    for (int j = 0; j < AE_HIDDEN_SIZE; j++) {
        float sum = _b1[j];
        for (int i = 0; i < AE_INPUT_SIZE; i++)
            sum += _w1[j][i] * norm[i];
        h1[j] = _sigmoid(sum);
    }

    // Encoder layer2 hidden to bottleneck
    for (int j = 0; j < AE_BOTTLENECK_SIZE; j++) {
        float sum = _b2[j];
        for (int i = 0; i < AE_HIDDEN_SIZE; i++)
            sum += _w2[j][i] * h1[i];
        bn[j] = _sigmoid(sum);
    }

    // Decoder layer1 bottleneck to hidden
    for (int j = 0; j < AE_HIDDEN_SIZE; j++) {
        float sum = _b3[j];
        for (int i = 0; i < AE_BOTTLENECK_SIZE; i++)
            sum += _w3[j][i] * bn[i];
        h2[j] = _sigmoid(sum);
    }

    // Decoder layer2 hidden to output
    for (int j = 0; j < AE_INPUT_SIZE; j++) {
        float sum = _b4[j];
        for (int i = 0; i < AE_HIDDEN_SIZE; i++)
            sum += _w4[j][i] * h2[i];
        out[j] = _sigmoid(sum);
    }
}

// Backpropagation
void Autoencoder::_backward(const float norm[AE_INPUT_SIZE],
                             const float h1[AE_HIDDEN_SIZE],
                             const float bn[AE_BOTTLENECK_SIZE],
                             const float h2[AE_HIDDEN_SIZE],
                             const float out[AE_INPUT_SIZE]) {
    // Output layer deltas, MSE loss gradient
    float dOut[AE_INPUT_SIZE];
    for (int i = 0; i < AE_INPUT_SIZE; i++)
        dOut[i] = (out[i] - norm[i]) * _sigmoidDeriv(out[i]);

    // Decoder hidden deltas
    float dH2[AE_HIDDEN_SIZE] = {};
    for (int i = 0; i < AE_HIDDEN_SIZE; i++) {
        float sum = 0.0f;
        for (int j = 0; j < AE_INPUT_SIZE; j++)
            sum += dOut[j] * _w4[j][i];
        dH2[i] = sum * _sigmoidDeriv(h2[i]);
    }

    // Bottleneck deltas
    float dBn[AE_BOTTLENECK_SIZE] = {};
    for (int i = 0; i < AE_BOTTLENECK_SIZE; i++) {
        float sum = 0.0f;
        for (int j = 0; j < AE_HIDDEN_SIZE; j++)
            sum += dH2[j] * _w3[j][i];
        dBn[i] = sum * _sigmoidDeriv(bn[i]);
    }

    // Encoder hidden deltas
    float dH1[AE_HIDDEN_SIZE] = {};
    for (int i = 0; i < AE_HIDDEN_SIZE; i++) {
        float sum = 0.0f;
        for (int j = 0; j < AE_BOTTLENECK_SIZE; j++)
            sum += dBn[j] * _w2[j][i];
        dH1[i] = sum * _sigmoidDeriv(h1[i]);
    }

    // Update weights for decoder layer2 (w4, b4)
    for (int j = 0; j < AE_INPUT_SIZE; j++) {
        for (int i = 0; i < AE_HIDDEN_SIZE; i++)
            _w4[j][i] -= AE_LEARNING_RATE * dOut[j] * h2[i];
        _b4[j] -= AE_LEARNING_RATE * dOut[j];
    }

    // Update weights for decoder layer1 (w3, b3)
    for (int j = 0; j < AE_HIDDEN_SIZE; j++) {
        for (int i = 0; i < AE_BOTTLENECK_SIZE; i++)
            _w3[j][i] -= AE_LEARNING_RATE * dH2[j] * bn[i];
        _b3[j] -= AE_LEARNING_RATE * dH2[j];
    }

    // ── Update weights for encoder layer2 (w2, b2)
    for (int j = 0; j < AE_BOTTLENECK_SIZE; j++) {
        for (int i = 0; i < AE_HIDDEN_SIZE; i++)
            _w2[j][i] -= AE_LEARNING_RATE * dBn[j] * h1[i];
        _b2[j] -= AE_LEARNING_RATE * dBn[j];
    }

    // ── Update weights for encoder layer1 (w1, b1)
    for (int j = 0; j < AE_HIDDEN_SIZE; j++) {
        for (int i = 0; i < AE_INPUT_SIZE; i++)
            _w1[j][i] -= AE_LEARNING_RATE * dH1[j] * norm[i];
        _b1[j] -= AE_LEARNING_RATE * dH1[j];
    }
}

// ── Private: helpers ───────────────────────────────────────────────────────

void Autoencoder::_normalise(const float raw[AE_INPUT_SIZE],
                              float norm[AE_INPUT_SIZE]) const {
    for (int i = 0; i < AE_INPUT_SIZE; i++) {
        float range = _maxV[i] - _minV[i];
        if (range < 1e-6f) {
            norm[i] = 0.5f;
        } else {
            norm[i] = (raw[i] - _minV[i]) / range;
            // Clamp to [0,1] so out-of-range sensors don't blow up the network
            norm[i] = constrain(norm[i], 0.0f, 1.0f);
        }
    }
}

float Autoencoder::_sigmoid(float x) const {
    return 1.0f / (1.0f + expf(-x));
}

float Autoencoder::_sigmoidDeriv(float y) const {
    // y is already sigmoid(x), so derivative = y*(1-y)
    return y * (1.0f - y);
}

void Autoencoder::_updateErrorStats(float err) {
    // Welford's online algorithm for mean and variance
    int n = _trainCount + 1;
    float delta  = err - _errMean;
    _errMean    += delta / n;
    float delta2 = err - _errMean;
    _errM2      += delta * delta2;
    _errVar      = (n > 1) ? (_errM2 / (n - 1)) : 0.0f;
}

void Autoencoder::_initWeights() {
    // Xavier initialisation: scale = sqrt(2 / fan_in)
    auto xavier = [](int fanIn) {
        return sqrtf(2.0f / fanIn);
    };

    float s;

    s = xavier(AE_INPUT_SIZE);
    for (int j = 0; j < AE_HIDDEN_SIZE;     j++)
        for (int i = 0; i < AE_INPUT_SIZE;  i++)
            _w1[j][i] = _randWeight(s);
    for (int j = 0; j < AE_HIDDEN_SIZE;     j++) _b1[j] = 0.0f;

    s = xavier(AE_HIDDEN_SIZE);
    for (int j = 0; j < AE_BOTTLENECK_SIZE; j++)
        for (int i = 0; i < AE_HIDDEN_SIZE; i++)
            _w2[j][i] = _randWeight(s);
    for (int j = 0; j < AE_BOTTLENECK_SIZE; j++) _b2[j] = 0.0f;

    s = xavier(AE_BOTTLENECK_SIZE);
    for (int j = 0; j < AE_HIDDEN_SIZE;          j++)
        for (int i = 0; i < AE_BOTTLENECK_SIZE;  i++)
            _w3[j][i] = _randWeight(s);
    for (int j = 0; j < AE_HIDDEN_SIZE;          j++) _b3[j] = 0.0f;

    s = xavier(AE_HIDDEN_SIZE);
    for (int j = 0; j < AE_INPUT_SIZE;      j++)
        for (int i = 0; i < AE_HIDDEN_SIZE; i++)
            _w4[j][i] = _randWeight(s);
    for (int j = 0; j < AE_INPUT_SIZE;      j++) _b4[j] = 0.0f;
}

float Autoencoder::_randWeight(float scale) {
    // LCG pseudo-random number generator (no stdlib rand needed)
    _rng = _rng * 1664525UL + 1013904223UL;
    float f = (float)(_rng >> 1) / (float)0x7FFFFFFF - 1.0f; // [-1, 1]
    return f * scale;
}
