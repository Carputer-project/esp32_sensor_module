#pragma once

// =============================================================================
// ESP32 Analog Sensor Module – Configuration
// Car ECU sensor inputs via ADC, output via UART (no CAN)
// =============================================================================

// --- ADC pin assignments (ADC1 only – safe to use alongside WiFi) -----------
// All GPIOs on ADC1: 32-39.  36/39 are input-only (no pull resistors needed).

#define PIN_MAP_SENSOR   36   // GPIO36 (VP) – Manifold Absolute Pressure
#define PIN_TPS_SENSOR   39   // GPIO39 (VN) – Throttle Position Sensor
#define PIN_CTS_SENSOR   34   // GPIO34      – Coolant Temperature Sensor
#define PIN_IAT_SENSOR   35   // GPIO35      – Intake Air Temperature
#define PIN_O2_SENSOR    32   // GPIO32      – O2 / Wideband Lambda
#define PIN_BATT_SENSOR  33   // GPIO33      – Battery Voltage

// --- ADC resolution & reference ---------------------------------------------
#define ADC_RESOLUTION    12           // bits (0–4095)
#define ADC_MAX_VALUE     4095         // 2^12 – 1
#define ADC_REF_VOLTAGE   3.3f         // ESP32 VREF (volts)
#define ADC_SAMPLES       16           // oversampling count per reading

// --- Voltage-divider ratios -------------------------------------------------
// 5 V sensors are scaled down to ≤3.3 V using a resistor divider.
// Ratio = R_low / (R_high + R_low).  Default: 10 kΩ / (10 kΩ + 5.6 kΩ).
#define VDIV_5V_RATIO   (10000.0f / (10000.0f + 5600.0f))   // ≈ 0.641

// Battery: 12 V nominal → 3.3 V.  Default: 10 kΩ / (47 kΩ + 10 kΩ).
#define VDIV_BATT_RATIO (10000.0f / (47000.0f + 10000.0f))  // ≈ 0.175

// --- MAP sensor calibration (2-bar / 200 kPa absolute sensor) ---------------
// Output 0.5 V at 10 kPa, 4.5 V at 310 kPa (linear).
#define MAP_V_MIN    0.5f    // volts at minimum pressure
#define MAP_V_MAX    4.5f    // volts at maximum pressure
#define MAP_KPA_MIN  10.0f   // kPa at MAP_V_MIN
#define MAP_KPA_MAX  310.0f  // kPa at MAP_V_MAX

// --- TPS calibration --------------------------------------------------------
// Potentiometer: 0.5 V fully closed, 4.5 V wide-open throttle (linear).
#define TPS_V_MIN   0.5f   // volts at 0 % throttle
#define TPS_V_MAX   4.5f   // volts at 100 % throttle

// --- NTC thermistor parameters (CTS & IAT) ----------------------------------
// Steinhart–Hart simplified (B-coefficient model).
// Common automotive NTC: 2.5 kΩ at 25 °C, B ≈ 3977 K.
#define NTC_NOMINAL_RES   2500.0f   // Ω at nominal temperature
#define NTC_NOMINAL_TEMP  25.0f     // °C
#define NTC_B_COEFF       3977.0f   // B coefficient (K)
#define NTC_SERIES_RES    2490.0f   // Series resistor (Ω)
#define NTC_SUPPLY_V      3.3f      // Divider supply voltage (V)

// --- O2 / Wideband Lambda calibration ---------------------------------------
// AEM UEGO / Innovate style: 0–5 V output = 0.5–1.52 λ (linear).
#define O2_V_MIN      0.0f   // volts
#define O2_V_MAX      5.0f   // volts
#define O2_LAMBDA_MIN 0.50f  // λ at O2_V_MIN
#define O2_LAMBDA_MAX 1.52f  // λ at O2_V_MAX

// --- Battery voltage --------------------------------------------------------
#define BATT_MAX_V  20.0f   // maximum expected voltage (V)

// --- Update rate ------------------------------------------------------------
#define UPDATE_RATE_MS  100   // sensor sampling period (ms) → 10 Hz

// --- UART communication -----------------------------------------------------
#define UART_BAUD_RATE  115200
