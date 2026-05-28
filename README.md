# AquaFusion: AI-Powered Edge IoT Environmental Monitor

A modular, multi-parameter IoT device for distributed deployment and dense monitoring across diverse freshwater environments. 

## Technical Architecture & Core Modules

* **Edge AI Anomaly Detection (`Autoencoder.cpp`)**: Runs an on-device neural network autoencoder to detect sensor anomalies and environmental contamination in real-time without cloud dependency.
* **Signal Filtering (`KalmanFilter.h`)**: Implements raw data smoothing to eliminate environmental noise from dynamic water currents before features are passed to the ML model.
* **Multi-Sensor Driver Pipeline**: Modular C++ architecture interfacing with pH, TDS, Turbidity, Depth, and Temperature arrays simultaneously.

## Hardware Stack
* **Microcontroller**: Raspberry Pi Pico
* **Language**: C++ / C (92% optimized for embedded performance)
