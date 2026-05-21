# esp32_sensor_module

An ESP32-based analog sensor module for car ECU applications. Reads six
common automotive sensors via the on-chip 12-bit ADC and streams live data as
JSON over UART at 10 Hz. **No CAN bus required.**

---

## Supported Sensors

| Channel | Sensor | Output | GPIO |
|---------|--------|--------|------|
| MAP | Manifold Absolute Pressure | 0.5–4.5 V (10–310 kPa) | 36 |
| TPS | Throttle Position Sensor | 0.5–4.5 V (0–100 %) | 39 |
| CTS | Coolant Temperature Sensor | NTC thermistor | 34 |
| IAT | Intake Air Temperature | NTC thermistor | 35 |
| O2  | Wideband Lambda / O2 | 0–5 V (λ 0.50–1.52) | 32 |
| BATT | Battery Voltage | 12 V nominal (via divider) | 33 |

---

## Hardware

### Voltage dividers for 5 V sensors

The ESP32 ADC inputs are limited to **3.3 V**. 5 V sensor outputs must be
scaled down with a resistor divider before reaching the GPIO pin.

Recommended values: **R_high = 5.6 kΩ, R_low = 10 kΩ** (ratio ≈ 0.641).

```
Sensor out (5 V max) ─── R_high ─┬─── GPIO (≤3.3 V)
                                  │
                                R_low
                                  │
                                GND
```

### Battery voltage divider

Battery voltage (up to ~16 V) is scaled with **R_high = 47 kΩ, R_low = 10 kΩ**
(ratio ≈ 0.175).

### NTC thermistors (CTS & IAT)

Connect the NTC between the GPIO pin and GND with a **2.49 kΩ** series resistor
from 3.3 V to the junction. The firmware uses the B-coefficient
Steinhart–Hart model (B = 3977 K, R₀ = 2.5 kΩ at 25 °C).

---

## UART Output

Data is transmitted on **UART0 (USB serial)** at **115200 baud**, one JSON
object per line, every 100 ms:

```json
{"map_kpa":95.20,"tps_pct":12.50,"cts_degc":85.10,"iat_degc":32.40,"o2_lambda":1.010,"batt_v":13.80,"valid":true}
```

| Field | Unit | Description |
|-------|------|-------------|
| `map_kpa` | kPa | Manifold absolute pressure |
| `tps_pct` | % | Throttle position (0 = closed, 100 = WOT) |
| `cts_degc` | °C | Coolant temperature |
| `iat_degc` | °C | Intake air temperature |
| `o2_lambda` | λ | Air/fuel ratio (1.0 = stoich) |
| `batt_v` | V | Battery / supply voltage |
| `valid` | bool | `false` if any reading is out of range |

Connect any UART-capable device (PC, Raspberry Pi, data logger) to the ESP32
USB or TX/RX pins to receive the stream.

---

## Building & Flashing

The project uses [PlatformIO](https://platformio.org/).

```bash
# Install PlatformIO CLI
pip install platformio

# Build firmware
pio run

# Flash to ESP32
pio run --target upload

# Open serial monitor
pio device monitor
```

---

## Project Structure

```
src/
  main.cpp        – Arduino setup() / loop()
  config.h        – Pin assignments, calibration constants
  sensors.h/.cpp  – ADC reading and unit conversion
  uart_comm.h/.cpp– JSON serialisation and UART output
platformio.ini    – PlatformIO build configuration
```

---

## Calibration

Edit `src/config.h` to match your specific sensors:

- `MAP_V_MIN / MAP_V_MAX / MAP_KPA_MIN / MAP_KPA_MAX` – MAP sensor curve
- `TPS_V_MIN / TPS_V_MAX` – throttle sensor end-stops
- `NTC_NOMINAL_RES / NTC_B_COEFF / NTC_SERIES_RES` – thermistor parameters
- `O2_V_MIN / O2_V_MAX / O2_LAMBDA_MIN / O2_LAMBDA_MAX` – wideband controller output
- `VDIV_5V_RATIO / VDIV_BATT_RATIO` – resistor divider ratios
- `UPDATE_RATE_MS` – sampling interval (default 100 ms = 10 Hz)
