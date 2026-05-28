#pragma once

/*
 * KalmanFilter.h
 * Lightweight 1D Kalman filter for Arduino / Pico.
 *
 * Tuning:
 *   measurementNoise (R) — how much you distrust the raw sensor.
 *                          Higher = smoother but slower to react.
 *   processNoise     (Q) — how fast the true value can change.
 *                          Higher = reacts faster, less smooth.
 *
 * Recommended starting values:
 *   TDS         R=5.0   Q=0.1
 *   Turbidity   R=4.0   Q=0.1
 *   pH          R=0.5   Q=0.01
 *   Temperature R=0.3   Q=0.01
 */
class KalmanFilter {
public:
    KalmanFilter(float measurementNoise, float processNoise)
        : _R(measurementNoise)
        , _Q(processNoise)
        , _x(0.0f)
        , _p(1.0f)
        , _seeded(false)
    {}

    // Feed a raw reading; returns smoothed estimate.
    // On the very first call it seeds itself with the raw value.
    float update(float measurement) {
        if (!_seeded) {
            _x = measurement;
            _seeded = true;
            return _x;
        }

        // Predict: uncertainty grows by process noise
        _p = _p + _Q;

        // Update: blend estimate with new measurement
        float k = _p / (_p + _R);   // Kalman gain
        _x = _x + k * (measurement - _x);
        _p = (1.0f - k) * _p;

        return _x;
    }

    float estimate() const { return _x; }

    void reset(float value) {
        _x = value;
        _p = 1.0f;
        _seeded = true;
    }

private:
    float _R, _Q, _x, _p;
    bool  _seeded;
};
