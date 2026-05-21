#include "sensors.h"
#include "config.h"

#include <Arduino.h>
#include <math.h>

// =============================================================================
// Internal helpers
// =============================================================================

// Oversample an ADC pin (ADC_SAMPLES readings) and return the averaged voltage.
static float read_voltage(uint8_t pin)
{
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i) {
        sum += analogRead(pin);
    }
    float adc_count = static_cast<float>(sum) / ADC_SAMPLES;
    return (adc_count / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
}

// Linear interpolation: map value from [in_min, in_max] → [out_min, out_max].
static float linear_map(float value, float in_min, float in_max,
                         float out_min, float out_max)
{
    if (in_max == in_min) return out_min;
    float ratio = (value - in_min) / (in_max - in_min);
    return out_min + ratio * (out_max - out_min);
}

// Convert NTC thermistor voltage to temperature (°C) using the B-coefficient
// Steinhart–Hart model.  The thermistor forms the lower leg of a voltage
// divider:  V_out = V_supply × R_ntc / (R_series + R_ntc)
static float ntc_to_celsius(float voltage)
{
    // Avoid division by zero if voltage is at the rail
    if (voltage <= 0.0f || voltage >= NTC_SUPPLY_V) return NAN;

    // Calculate thermistor resistance from measured voltage
    float r_ntc = NTC_SERIES_RES * voltage / (NTC_SUPPLY_V - voltage);

    // B-coefficient model: 1/T = 1/T0 + (1/B) * ln(R/R0)
    float t0_k   = NTC_NOMINAL_TEMP + 273.15f;
    float t_k    = 1.0f / (1.0f / t0_k + (1.0f / NTC_B_COEFF) * logf(r_ntc / NTC_NOMINAL_RES));
    return t_k - 273.15f;
}

// =============================================================================
// Public API
// =============================================================================

void sensors_init()
{
    analogReadResolution(ADC_RESOLUTION);
    analogSetAttenuation(ADC_11db);   // full 0–3.3 V input range

    // Configure ADC1 input pins
    pinMode(PIN_MAP_SENSOR,  INPUT);
    pinMode(PIN_TPS_SENSOR,  INPUT);
    pinMode(PIN_CTS_SENSOR,  INPUT);
    pinMode(PIN_IAT_SENSOR,  INPUT);
    pinMode(PIN_O2_SENSOR,   INPUT);
    pinMode(PIN_BATT_SENSOR, INPUT);
}

SensorData sensors_read()
{
    SensorData data;
    data.valid = true;

    // --- MAP sensor ---------------------------------------------------------
    // 5 V sensor attenuated to ≤3.3 V by voltage divider.
    float map_v_div = read_voltage(PIN_MAP_SENSOR);
    float map_v     = map_v_div / VDIV_5V_RATIO;   // reconstruct sensor voltage
    data.map_kpa    = linear_map(map_v, MAP_V_MIN, MAP_V_MAX, MAP_KPA_MIN, MAP_KPA_MAX);
    if (data.map_kpa < MAP_KPA_MIN || data.map_kpa > MAP_KPA_MAX) data.valid = false;

    // --- TPS ----------------------------------------------------------------
    float tps_v_div = read_voltage(PIN_TPS_SENSOR);
    float tps_v     = tps_v_div / VDIV_5V_RATIO;
    data.tps_pct    = linear_map(tps_v, TPS_V_MIN, TPS_V_MAX, 0.0f, 100.0f);
    data.tps_pct    = fmaxf(0.0f, fminf(100.0f, data.tps_pct));   // clamp 0–100 %

    // --- Coolant Temperature ------------------------------------------------
    float cts_v     = read_voltage(PIN_CTS_SENSOR);
    data.cts_degc   = ntc_to_celsius(cts_v);
    if (isnan(data.cts_degc)) data.valid = false;

    // --- Intake Air Temperature ---------------------------------------------
    float iat_v     = read_voltage(PIN_IAT_SENSOR);
    data.iat_degc   = ntc_to_celsius(iat_v);
    if (isnan(data.iat_degc)) data.valid = false;

    // --- O2 / Wideband Lambda -----------------------------------------------
    // 5 V sensor attenuated to ≤3.3 V.
    float o2_v_div  = read_voltage(PIN_O2_SENSOR);
    float o2_v      = o2_v_div / VDIV_5V_RATIO;
    data.o2_lambda  = linear_map(o2_v, O2_V_MIN, O2_V_MAX, O2_LAMBDA_MIN, O2_LAMBDA_MAX);

    // --- Battery Voltage ----------------------------------------------------
    float batt_v_div = read_voltage(PIN_BATT_SENSOR);
    data.batt_v      = batt_v_div / VDIV_BATT_RATIO;
    if (data.batt_v > BATT_MAX_V) data.valid = false;

    return data;
}
