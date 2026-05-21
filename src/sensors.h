#pragma once

#include <stdint.h>

// =============================================================================
// Sensor module interface
// Reads car ECU analog sensors via the ESP32 ADC and converts raw counts to
// engineering units.  All communication uses UART – no CAN bus required.
// =============================================================================

// Aggregated sensor data snapshot
struct SensorData {
    float map_kpa;      // Manifold Absolute Pressure  (kPa)
    float tps_pct;      // Throttle Position            (%)
    float cts_degc;     // Coolant Temperature          (°C)
    float iat_degc;     // Intake Air Temperature       (°C)
    float o2_lambda;    // O2 / Wideband Lambda         (λ)
    float batt_v;       // Battery Voltage              (V)
    bool  valid;        // true when all readings are in range
};

// Initialise ADC channels.  Call once from setup().
void sensors_init();

// Read all sensors and return a populated SensorData struct.
SensorData sensors_read();
