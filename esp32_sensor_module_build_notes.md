# esp32_sensor_module - Build Notes

## Overview
Dual ESP32-WROOM-32 bare module PCB (130×80mm) for engine ECU sensor interface (example: Toyota 5S-FE)
interface (J5) + body controller with ULN2003A relays (J6).

## Circuits
- **6× voltage dividers** (5.6K+10K THT axial) for: PIM, THW, VTA, SPD, IGf, T/VF
- **2× 120Ω pullups** for fuel level & oil pressure senders
- **4× 10K series resistors** for door/hood/trunk GND-switch inputs
- **2× 2N2222 + 1K base R** for TE1/TE2 DTC reading
- **1× 10K pulldown** on GPIO39 (VN) for ADC crosstalk prevention
- **DC-DC converter** from 12V → 3.3V
- **2× ULN2003A** DIP-16 relay drivers (10 channels)
- **Flyback diodes** (9× 1N4007) for relay coil suppression

## J5 Sensor WROOM-32 Signal Map (J5)
| Signal | GPIO | Module Pin |
|--------|------|------------|
| DRV | 13 | 20 |
| PASS | 14 | 29 |
| TVF | 19 |  8 |
| HOOD | 21 |  6 |
| TE1 | 22 |  3 |
| TE2 | 23 |  2 |
| SPD | 25 | 26 |
| IGF | 26 | 27 |
| TRUNK | 27 | 28 |
| PIM | 32 | 30 |
| THW | 33 | 31 |
| VTA | 34 | 34 |
| FUEL | 35 | 33 |
| OIL | 36 | 36 |
| VN_PD | 39 | 35 |

## J6 Body WROOM-32 Signal Map (J6)
| Signal | GPIO | Module Pin |
|--------|------|------------|
| HVAC |  2 | 15 |
| AC |  4 | 13 |
| REMOTE |  5 | 10 |
| FAN2 | 12 | 24 |
| FAN1 | 18 |  9 |
| BTN_SEL | 22 |  3 |
| BTN_EXIT | 23 |  2 |
| DOOR_LOCK | 25 | 26 |
| WINDOW_UP | 26 | 27 |
| WINDOW_DOWN | 27 | 28 |
| EXTRA1 | 32 | 30 |
| EXTRA2 | 33 | 31 |
| JOY_Y | 36 | 36 |
| JOY_X | 39 | 35 |

## ESP32-WROOM-32 Module Pinout (38-pin castellated)
| Pin | Function |
|-----|----------|
|  1 | GND              |
|  2 | GPIO23           |
|  3 | GPIO22           |
|  4 | GPIO1(TXD)       |
|  5 | GPIO3(RXD)       |
|  6 | GPIO21           |
|  7 | GND              |
|  8 | GPIO19           |
|  9 | GPIO18           |
| 10 | GPIO5            |
| 11 | GPIO17           |
| 12 | GPIO16           |
| 13 | GPIO4            |
| 14 | GPIO0            |
| 15 | GPIO2            |
| 16 | GPIO15           |
| 17 | GPIO8            |
| 18 | GPIO7            |
| 19 | GPIO6            |
| 20 | GPIO13           |
| 21 | GPIO9(SD2)       |
| 22 | GPIO10(SD3)      |
| 23 | GPIO11(CMD)      |
| 24 | GPIO12           |
| 25 | VDD              |
| 26 | GPIO25           |
| 27 | GPIO26           |
| 28 | GPIO27           |
| 29 | GPIO14           |
| 30 | GPIO32           |
| 31 | GPIO33           |
| 32 | GND              |
| 33 | GPIO35           |
| 34 | GPIO34           |
| 35 | GPIO39(VN)       |
| 36 | GPIO36(VP)       |
| 37 | EN               |
| 38 | NC               |

Pins 1-15 = left column (closer to USB, with USB up).
Pins 16-30 = right column (away from USB).
**GPIOs not on headers:** 6,7,8,9,10,11 (internal flash).
**Strapping:** GPIO0,2,5,12,15 — boot-sensitive.
**Input-only:** GPIO34,35,36(VP),39(VN).

## Power
- **Input:** 12V from ECU +B (fused F1 1A) via J4
- **DC-DC:** 12V→3.3V (U1), feeds all pullups and circuits
- **DevKit V1 power:** Via USB, or jumper from PCB 3.3V rail to DevKit's 3V3 pin
- **Filtering:** C1/C3 (0.1µF) + C2/C4 (10µF) per module region
- **Ground:** Common ground, bottom layer pour recommended

## Board Details
- **Size:** 130×80mm
- **Layers:** 2 (signal + GND pour on bottom)
- **Thickness:** 1.6mm FR-4
- **Mounting:** 4× M3 holes at corners
- **Cu weight:** 1oz (2oz recommended for high-current)
- **Tracks:** 0.25mm clearance, 0.6mm via dia, 0.3mm via drill

## Notes
- GPIOs 34/35/36/39 are input-only (no pullups internally).
- ULN2003A COM pin (9) → +12V for internal flyback diodes.
- Strapping pins on body module: GPIO2(HVAC), GPIO5(REMOTE),
  GPIO12(FAN2) — ensure peripherals don't interfere with boot.
