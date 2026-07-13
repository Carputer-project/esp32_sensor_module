# ESP32 Sensor Module

**GitHub**: [Carputer-project/esp32_sensor_module](https://github.com/Carputer-project/esp32_sensor_module)

Firmware for the sensor ESP32 on the Carputer PCB. Designed for **OBD1 vehicles** (pre-1996, all makes) — reads engine ECU sensor voltages, switch signals, and diagnostic pulses directly. No CAN bus required. CAN/OBD2 is an easy add — just a firmware update and an external CAN module (e.g. MCP2515 via SPI).

Currently configured for a Toyota 5S-FE as reference, but the voltage divider values and input pins work with any OBD1 ECU. Streams data over UDP to the carputer head unit.

## Role

Connects to ECU as WiFi STA at `192.168.4.20`, **broadcasts** JSON sensor data to `192.168.4.255:5001` (subnet broadcast so both carputer app and cluster display receive data). Also handles DTC (diagnostic trouble code) reading via the check-engine light blink pattern.

## ECU Signal Wiring (5.6K + 10K Voltage Divider)

All ECU signals go through a voltage divider: **R1 = 5.6KΩ** (series from ECU), **R2 = 10KΩ** (to GND). Ratio = 0.641x. Max output: 5V × 0.641 = 3.21V (safe under 3.3V).

```
ECU Signal ──[5.6KΩ]──┬── ESP32 GPIO
                        │
                       [10KΩ]
                        │
                       GND
```

### Analog Signals

| GPIO | ECU Signal | ECU Pin Name | ADC Channel | Unit | Notes |
|------|-----------|--------------|-------------|------|-------|
| GPIO 32 | PIM | MAP (Manifold Absolute Pressure) | ADC1_CH4 | kPa | 0-5V → 0-3.21V |
| GPIO 33 | THW | ECT (Engine Coolant Temp) | ADC1_CH5 | °F | NTC via ECU 2.2K pullup to 5V, reversed through divider |
| GPIO 34 | VTA | TPS (Throttle Position Sensor) | ADC1_CH6 | % | Input-only pin |

### Digital Pulse Signals

| GPIO | ECU Signal | ECU Pin Name | Edge | Calculation |
|------|-----------|--------------|------|-------------|
| GPIO 25 | SPD | Speed Sensor | FALLING | 1.985 meters/pulse |
| GPIO 26 | IGf | Ignition Feedback (RPM) | RISING | RPM = (pulses × 1000 / elapsed_ms × 60) / 2 (4-cyl) |

### DTC Reader

| GPIO | Function | Wiring | Notes |
|------|----------|--------|-------|
| GPIO 19 | T/VF read | 5.6K+10K divider from ECU T pin | MIL blink pattern, normally 5V |
| GPIO 22 | TE1 control | 1KΩ → base of 2N2222 NPN, collector → E1 (ECU GND), emitter → GND | Pulled HIGH = TE1 → E1 |
| GPIO 23 | TE2 control | 1KΩ → base of 2N2222 NPN, collector → E1 (ECU GND), emitter → GND | Pulled HIGH = TE2 → E1 |

### Additional Analog Inputs

| GPIO | Sensor | Circuit | Range | Notes |
|------|--------|---------|-------|-------|
| GPIO 35 | Fuel Level | 120Ω pullup to 3.3V, sender to GND | 3Ω (full) to 110Ω (empty) | ADC1_CH7, inverted logic |
| GPIO 36 | Oil Pressure | 120Ω pullup to 3.3V, sender to GND | 3Ω (0 psi) to 100Ω (100 psi) | ADC1_CH0 |
| GPIO 39 | Narrowband O2 | **Direct 0-1V, no divider** | 0.1V (rich 10:1) to 0.9V (lean 17:1) | ~450mV = stoich 14.7:1 |

### Door Switches (GND-Switched)

Wired with **10KΩ series resistor** from switch to GPIO. Uses `INPUT_PULLUP`.

| GPIO | Sensor | Logic |
|------|--------|-------|
| GPIO 13 | Driver Door | LOW = open, HIGH = closed |
| GPIO 14 | Passenger Door | LOW = open, HIGH = closed |
| GPIO 27 | Trunk | LOW = open, HIGH = closed |
| GPIO 21 | Hood | LOW = open, HIGH = closed |

### HC-05 Bluetooth Module (UART2 Serial Bridge)

| ESP32 Pin | HC-05 Pin | Function |
|-----------|-----------|----------|
| GPIO 16 (RX2) | TXD | Receives data FROM HC-05 |
| GPIO 17 (TX2) | RXD | Sends data TO HC-05 |
| GPIO 4 | STATE | HIGH = BT connected, LOW = idle |
| 5V | VCC | Power |
| GND | GND | Ground |

- Baud rate: 38400 (AT command mode)
- Serial bridge UDP port: 5004
- Used for ELM327 Bluetooth OBD2 adapter communication

## Network Configuration

| Parameter | Value |
|-----------|-------|
| WiFi SSID | Carputer_ECU |
| WiFi Password | 12345678 |
| WiFi Mode | STA (station) |
| Static IP | 192.168.4.20 |
| Gateway | 192.168.4.1 |
| Subnet | 255.255.255.0 |
| **UDP send** | **192.168.4.255:5001 (subnet broadcast)** |
| UDP listen (commands) | 5002 |
| UDP DTC results | 5003 |
| UDP serial bridge | 5004 (HC-05/ELM327) |
| Sensor update rate | 500ms |
| Speed/RPM calc rate | 100ms |

## Protocol

### Sensor Data (UDP 5001, broadcast)
```json
{
  "event": "sensors",
  "data": {
    "coolant": 185,
    "oil": 212,
    "ambient": 72,
    "intake": 68,
    "rpm": 2450,
    "speed": 45,
    "tps": 23,
    "map": 65,
    "fuel": 42,
    "oilPress": 38,
    "afr": 14.7,
    "driverDoor": 0,
    "passDoor": 0,
    "trunk": 0,
    "hood": 0,
    "obd2FuelTrim": 0,
    "obd2Load": 0,
    "obd2Coolant": 0,
    "obd2Rpm": 0,
    "obd2Speed": 0,
    "obd2FuelPress": 0,
    "obd2DistDtc": 0
  }
}
```

### Commands (UDP 5002)
```json
{"cmd": "query"}
```

### DTC (UDP 5003)
- Send query → reads CEL blink pattern via T/VF pin
- Returns parsed DTC codes

## Complete Pin Allocation

| GPIO | Function | Type | Notes |
|------|----------|------|-------|
| 2 | LED indicator | OUTPUT | Build status |
| 4 | HC-05 STATE | INPUT | BT connection status |
| 12 | Door switch (old) | INPUT_PULLUP | Reassigned — now used in body controller |
| 13 | Driver Door | INPUT_PULLUP | 10K series resistor |
| 14 | Passenger Door | INPUT_PULLUP | 10K series resistor |
| 16 | UART2 RX (HC-05 TXD) | Serial RX2 | 38400 baud |
| 17 | UART2 TX (HC-05 RXD) | Serial TX2 | 38400 baud |
| 19 | DTC T/VF read | INPUT | 5.6K+10K divider |
| 21 | Hood switch | INPUT_PULLUP | 10K series resistor |
| 22 | TE1 control | OUTPUT | 1K → 2N2222 base |
| 23 | TE2 control | OUTPUT | 1K → 2N2222 base |
| 25 | Speed (SPD) | Digital interrupt | FALLING edge |
| 26 | RPM (IGf) | Digital interrupt | RISING edge |
| 27 | Trunk switch | INPUT_PULLUP | 10K series resistor |
| 32 | MAP (PIM) | ADC1_CH4 | 5.6K+10K divider |
| 33 | Coolant (THW) | ADC1_CH5 | 5.6K+10K divider |
| 34 | TPS (VTA) | ADC1_CH6 | 5.6K+10K divider (input-only) |
| 35 | Fuel level | ADC1_CH7 | 120Ω pullup |
| 36 | Oil pressure | ADC1_CH0 | 120Ω pullup |
| 39 | Narrowband O2 | ADC1_CH3 | Direct 0-1V, no divider |

## Parts List

| Component | Value | Quantity | Purpose |
|-----------|-------|----------|---------|
| Resistor | 5.6KΩ 1/4W | 6 | Voltage divider R1 (ECU signals + T/VF) |
| Resistor | 10KΩ 1/4W | 11 | Voltage divider R2 (6) + door pullups (4) + GPIO39 fix (1) |
| Resistor | 1KΩ 1/4W | 6 | Series protection (4 doors + 2 TE switch bases) |
| Resistor | 120Ω 1/4W | 2 | Fuel level + oil pressure sender pullups |
| 2N2222 NPN transistor | — | 2 | TE1/TE2 open-drain switches to ECU E1 |
| DC-DC converter | Adjustable → 3.3V | 1 | Powers ESP32 from 12V |
| ESP32 DevKit | — | 1 | Sensor module MCU |

## Building & Flashing

```bash
cd esp32_sensor_module
pio run                    # Build
pio run --target upload    # Upload via USB
pio device monitor -b 115200  # Serial monitor
```

## Related Repositories

| Repo | Role |
|------|------|
| [carputer](https://github.com/Carputer-project/carputer) | Qt5 head unit — receives UDP sensor data |
| [carputer-pcb](https://github.com/Carputer-project/carputer-pcb) | Physical PCB with J5 socket for this ESP32 |
| [carputerandroid](https://github.com/Carputer-project/carputerandroid) | Android head unit — receives same UDP data |
| [esp32_body_controller](https://github.com/Carputer-project/esp32_body_controller) | Creates Carputer_ECU WiFi AP, forwards sensor data |
| [esp32_cluster](https://github.com/Carputer-project/esp32_cluster) | TFT display — receives broadcast sensor data on UDP 5001 |
