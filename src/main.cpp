#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include "bind.h"

// WiFi configuration - connect to Body Controller AP
const char* WIFI_SSID = "Carputer_ECU";
const char* WIFI_PASSWORD = "12345678";

// Carputer IP and ports
const IPAddress CARPUTER_IP(192, 168, 4, 3);
const int SENSOR_PORT = 5001;       // Send sensor data TO carputer
const int CMD_PORT = 5002;          // Listen for commands FROM carputer
const int DTC_PORT = 5003;          // Send DTC results TO carputer

WiFiUDP udp;

// ── ECU-tapped signals (with external 5.6K+10K voltage dividers) ───────
// ADC1 analog inputs (0-5V sensor → divider → 0-3.3V to ESP32)
const int PIN_MAP   = 32;   // PIM - manifold absolute pressure
const int PIN_THW   = 33;   // ECT - engine coolant temp (NTC via ECU pullup)
const int PIN_VTA   = 34;   // TPS - throttle position sensor

// Digital pulse inputs (with voltage dividers if 5V)
const int PIN_SPD   = 25;   // SPD - vehicle speed sensor (interrupt)
const int PIN_IGF   = 26;   // IGf - ignition feedback for RPM (interrupt)

// ── Fuel and oil pressure (120Ω pullup to 3.3V, sender variable R to GND) ─
const int PIN_FUEL      = 35;   // ADC1_CH7 — fuel level sender, 3Ω full ~ 110Ω empty
const int PIN_OIL_PRESS = 36;   // ADC1_CH0 — oil pressure sender, 3Ω at 0psi ~ 100Ω at 100psi

// ── Narrowband O2 sensor (0-1V direct, no divider needed) ────────────────
const int PIN_O2 = 39;    // ADC1_CH3 — narrowband O2, 0-1V swing, ~450mV at stoich

// ── HC-05 Bluetooth module (UART2) ──────────────────────────────────────
const int PIN_HC05_TX   = 16;   // TX2 → HC-05 RXD (HC-05 TXD = received data)
const int PIN_HC05_RX   = 17;   // RX2 ← HC-05 TXD
const int PIN_HC05_STATE = 4;   // INPUT: HIGH when connected, LOW when idle
const int HC05_BAUD = 38400;    // AT command mode baud rate (default)
const int SERIAL_BRIDGE_PORT = 5004;  // UDP port for serial data bridge

// ── Door/switch sensors (GND-switched, INPUT_PULLUP + 1K series) ─────
const int PIN_DRIVER_DOOR     = 13;
const int PIN_PASSENGER_DOOR  = 14;
const int PIN_TRUNK           = 27;
const int PIN_HOOD            = 21;

// ── DTC read pins ─────────────────────────────────────────────────────
const int PIN_TE1 = 22;   // Output: transistor to ECU TE1 (pull to E1)
const int PIN_TE2 = 23;   // Output: transistor to ECU TE2 (pull to E1)
const int PIN_TVF = 19;   // Input:  ECU T/VF signal (MIL blink, via 5.6K+10K divider)

// ── Moving average buffers ────────────────────────────────────────────
const int AVG_SAMPLES = 8;
float coolantSamples[AVG_SAMPLES];
float mapSamples[AVG_SAMPLES];
float throttleSamples[AVG_SAMPLES];
float fuelSamples[AVG_SAMPLES];
float oilPressSamples[AVG_SAMPLES];
float o2Samples[AVG_SAMPLES];
int sampleIndex = 0;

// ── Speed sensor ──────────────────────────────────────────────────────
volatile unsigned long speedPulses = 0;
unsigned long lastSpeedUpdate = 0;
float currentSpeed = 0;
const float METERS_PER_PULSE = 1.985;

void IRAM_ATTR speedPulse() { speedPulses++; }

// ── RPM sensor (IGf) ─────────────────────────────────────────────────
volatile unsigned long igfPulses = 0;
unsigned long lastRpmUpdate = 0;
float currentRpm = 0;

void IRAM_ATTR igfPulse() { igfPulses++; }

// ── Conversion functions ──────────────────────────────────────────────

float ntcFromEcu(int adcValue) {
    if (adcValue <= 0) return -99.0;
    float vAdc  = (float)adcValue / 4095.0 * 3.3;
    float vThw  = vAdc / 0.641;
    if (vThw >= 5.0) return -99.0;
    if (vThw <= 0.0) return 220.0;
    float rNtc  = 2200.0 * vThw / (5.0 - vThw);
    if (rNtc <= 0) return 220.0;
    float tempK = 1.0 / (1.0 / 298.15 + log(rNtc / 10000.0) / 3435.0);
    return (tempK - 273.15) * 9.0 / 5.0 + 32.0;
}

float readMapPercent(int adcValue) {
    float vAdc = (float)adcValue / 4095.0 * 3.3;
    float vPim = vAdc / 0.641;
    float pct = (vPim - 0.5) / 4.0 * 100.0;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

float readThrottlePercent(int adcValue) {
    float vAdc = (float)adcValue / 4095.0 * 3.3;
    float vVta = vAdc / 0.641;
    float pct = (vVta - 0.5) / 4.0 * 100.0;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

float readSenderPercent(int adcValue, float rMin, float rMax, bool invert) {
    float vAdc = (float)adcValue / 4095.0 * 3.3;
    if (vAdc >= 3.28) return invert ? 0.0 : 100.0;
    if (vAdc <= 0.01) return invert ? 100.0 : 0.0;
    float rSender = 120.0 * vAdc / (3.3 - vAdc);
    if (rSender < rMin) rSender = rMin;
    if (rSender > rMax) rSender = rMax;
    float pct = (rSender - rMin) / (rMax - rMin) * 100.0;
    return invert ? 100.0 - pct : pct;
}

// Narrowband O2 → AFR (nonlinear around stoich)
// 0.1V ≈ 11:1 (rich), 0.45V ≈ 14.7:1 (stoich), 0.9V ≈ 17:1 (lean)
float o2ToAfr(int adcValue) {
    float v = (float)adcValue / 4095.0 * 3.3;
    if (v < 0.05) return 10.0;
    if (v > 1.2) return 20.0;
    if (v < 0.45)
        return 14.7 - (0.45 - v) / 0.40 * 4.7;
    else
        return 14.7 + (v - 0.45) / 0.55 * 3.3;
}

void initSamples() {
    for (int i = 0; i < AVG_SAMPLES; i++) {
        coolantSamples[i] = 0;
        mapSamples[i] = 0;
        throttleSamples[i] = 0;
        fuelSamples[i] = 0;
        oilPressSamples[i] = 0;
        o2Samples[i] = 0;
    }
}

float movingAverage(float* samples, float newValue) {
    samples[sampleIndex % AVG_SAMPLES] = newValue;
    float sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) sum += samples[i];
    return sum / AVG_SAMPLES;
}

// ── DTC State Machine ─────────────────────────────────────────────────
enum DtcState {
    DTC_IDLE,
    DTC_WAIT_START,       // TE1 pulled low, waiting ~2s for ECU to respond
    DTC_READ_BLINK,       // Reading blink pattern from T/VF pin
    DTC_SEND_RESULT,      // Sending results via UDP
    DTC_CLEANUP           // Release pins
};

DtcState dtcState = DTC_IDLE;
bool dtcTestMode = false;
unsigned long dtcTimer = 0;

// Blink decoding state
bool firstBlink = true;
int digit1blinks = 0;       // blink count for digit 1 of current code
int digit2blinks = 0;       // blink count for digit 2 of current code
bool onSecondDigit = false; // true = currently counting digit 2, false = digit 1
int dtcCodes[10];
int dtcCodeCount = 0;
unsigned long lastTvfEdge = 0;
int lastTvf = LOW;
int consecutiveBlinks = 0;  // blink count for current digit
unsigned long readStartTime = 0;
bool noCodesDetected = true;

// ── OBD2 data (populated by app via serial bridge, included in sensor JSON) ─
int obd2Rpm = 0;
int obd2Speed = 0;
int obd2Load = 0;
int obd2Coolant = 0;
int obd2Intake = 0;
int obd2Timing = 0;
int obd2FuelTrimS = 0;
int obd2FuelTrimL = 0;
int obd2FuelPress = 0;
int obd2FuelSystem = 0;
int obd2DistDtc = 0;

// ── HC-05 serial bridge ────────────────────────────────────────────────
char serialBridgeBuf[512];
int serialBridgeLen = 0;
unsigned long lastSerialBridgeSend = 0;

// ── State ─────────────────────────────────────────────────────────────
unsigned long lastUpdate = 0;
unsigned long lastWifiCheck = 0;

// ── Module binding ────────────────────────────────────────────────────
ModuleBinder binder;

// ── UDP command buffer ────────────────────────────────────────────────
char packetBuffer[256];

// ── DTC Helpers ───────────────────────────────────────────────────────

void dtcResetBlinkState() {
    firstBlink = true;
    digit1blinks = 0;
    digit2blinks = 0;
    onSecondDigit = false;
    consecutiveBlinks = 0;
    dtcCodeCount = 0;
    lastTvfEdge = 0;
    lastTvf = LOW;
    noCodesDetected = true;
}

void dtcStartRead(bool testMode) {
    if (dtcState != DTC_IDLE) return;
    dtcTestMode = testMode;
    dtcResetBlinkState();
    dtcState = DTC_WAIT_START;
    dtcTimer = millis();
    readStartTime = 0;

    if (dtcTestMode) {
        digitalWrite(PIN_TE2, HIGH);
        digitalWrite(PIN_TE1, LOW);
        Serial.println("[DTC] Test mode: TE2+E1 connected");
    } else {
        digitalWrite(PIN_TE1, HIGH);
        digitalWrite(PIN_TE2, LOW);
        Serial.println("[DTC] Normal mode: TE1+E1 connected");
    }
}

void dtcCleanup() {
    digitalWrite(PIN_TE1, LOW);
    digitalWrite(PIN_TE2, LOW);
    dtcState = DTC_IDLE;
    Serial.println("[DTC] Pins released");
}

void dtcStoreCode() {
    if (digit1blinks == 0 && consecutiveBlinks == 0) return; // no code
    int d1 = digit1blinks;
    int d2 = onSecondDigit ? consecutiveBlinks : 0;
    int code = d1 * 10 + d2;
    if (code > 0 && code < 100 && dtcCodeCount < 10) {
        dtcCodes[dtcCodeCount++] = code;
        Serial.printf("[DTC] Code %d (%d-%d)\n", code, d1, d2);
    }
}

void dtcSendResult() {
    StaticJsonDocument<384> doc;
    doc["event"] = "dtc";
    doc["mode"] = dtcTestMode ? "test" : "normal";
    doc["count"] = dtcCodeCount;

    if (dtcCodeCount == 0 && noCodesDetected) {
        doc.createNestedArray("codes");
        doc["status"] = "no_codes";
    } else {
        JsonArray arr = doc.createNestedArray("codes");
        for (int i = 0; i < dtcCodeCount; i++) arr.add(dtcCodes[i]);
        doc["status"] = "ok";
    }

    udp.beginPacket(CARPUTER_IP, DTC_PORT);
    serializeJson(doc, udp);
    udp.println();
    udp.endPacket();
    Serial.print("[DTC] Sent: ");
    serializeJson(doc, Serial);
    Serial.println();
}

// ── Blink Pattern Reader ──────────────────────────────────────────────
// Called every loop iteration while DTC_READ_BLINK is active.
// Reads T/VF pin and decodes the MIL blink pattern.
//
// Timing (from factory manual FI-41):
//   Normal (no codes): 0.26s HIGH, 0.26s LOW (alternating)
//   Code blink:        ~0.5s HIGH per blink
//   Gap between digits: ~1.5s LOW
//   Gap between codes:  ~2.5s LOW
//   Repeat pause:       ~4.5s LOW
//
// We use the LOW period duration on each rising edge to determine context.

void runBlinkReader(unsigned long now) {
    int tvf = digitalRead(PIN_TVF);

    // Detect rising edge (LOW → HIGH) — end of a LOW period, next blink starts
    if (tvf == HIGH && lastTvf == LOW) {
        unsigned long lowDuration = now - lastTvfEdge;

        if (lowDuration > 100) {  // Ignore noise
            if (noCodesDetected) noCodesDetected = false;

            if (firstBlink) {
                // Very first rising edge after ECU starts outputting
                firstBlink = false;
                consecutiveBlinks = 1;
                readStartTime = now;
            } else if (lowDuration < 800) {
                // Short LOW (~500ms) → same digit, count another blink
                consecutiveBlinks++;
            } else if (lowDuration < 2000) {
                // Medium LOW (~1.5s) → digit gap
                if (!onSecondDigit) {
                    digit1blinks = consecutiveBlinks;
                    onSecondDigit = true;
                } else {
                    // Shouldn't happen: digit gap on digit 2 means code end
                    digit2blinks = consecutiveBlinks;
                    dtcStoreCode();
                    // Reset for next code
                    digit1blinks = digit2blinks = 0;
                    onSecondDigit = false;
                }
                consecutiveBlinks = 1; // This rising edge is blink 1 of next part
            } else if (lowDuration < 3500) {
                // Long LOW (~2.5s) → code gap
                if (onSecondDigit) {
                    digit2blinks = consecutiveBlinks;
                    dtcStoreCode();
                } else {
                    // Code with only digit 1? Store as-is
                    dtcStoreCode();
                }
                digit1blinks = digit2blinks = 0;
                onSecondDigit = false;
                consecutiveBlinks = 1; // This rising edge is blink 1 of next code
            } else {
                // Very long LOW (≥3.5s) → repeat detected, all codes read
                if (onSecondDigit) {
                    digit2blinks = consecutiveBlinks;
                    dtcStoreCode();
                }
                // Last code already stored from code gap, or stored here from repeat
                noCodesDetected = false;
                dtcState = DTC_SEND_RESULT;
                dtcTimer = now;
                lastTvf = tvf;
                lastTvfEdge = now;
                return;
            }
        }

        lastTvfEdge = now;
        lastTvf = tvf;
        return;
    }

    // Detect falling edge (HIGH → LOW) — end of a blink
    if (tvf == LOW && lastTvf == HIGH) {
        unsigned long highDuration = now - lastTvfEdge;

        // Check for normal (no codes) pattern: 0.26s HIGH
        // If we've been reading for >3s and all high durations are ~260ms, it's no-codes
        if (consecutiveBlinks > 10 && noCodesDetected && highDuration < 350) {
            unsigned long readElapsed = now - readStartTime;
            if (readElapsed > 3000) {
                // Been reading the alternating pattern for 3+ seconds — no codes
                dtcState = DTC_SEND_RESULT;
                dtcTimer = now;
                lastTvf = tvf;
                lastTvfEdge = now;
                return;
            }
        }

        lastTvfEdge = now;
        lastTvf = tvf;
        return;
    }

    // No edge — just track if we've been reading too long
    if (!firstBlink && dtcState == DTC_READ_BLINK) {
        unsigned long elapsed = now - readStartTime;
        if (elapsed > 15000) {  // 15s timeout
            Serial.println("[DTC] Read timeout");
            dtcState = DTC_SEND_RESULT;
            dtcTimer = now;
        }
    }

    lastTvf = tvf;
}

// ── DTC State Machine Runner ──────────────────────────────────────────
void runDtcStateMachine(unsigned long now) {
    switch (dtcState) {
        case DTC_IDLE:
            // Check for incoming UDP commands
            break;

        case DTC_WAIT_START: {
            unsigned long elapsed = now - dtcTimer;

            if (dtcTestMode) {
                // Test mode: TE2 already pulled, need to wait for TE1 after 2s
                if (elapsed < 2000) break; // wait 2s

                // Now also pull TE1 low
                digitalWrite(PIN_TE1, HIGH);
                dtcTimer = now;
                Serial.println("[DTC] Test mode: TE1+E1 also connected, reading...");

                // Fall through to reading phase
                dtcState = DTC_READ_BLINK;
                dtcTimer = now;
                lastTvf = digitalRead(PIN_TVF);
                lastTvfEdge = now;
                readStartTime = now;
                Serial.println("[DTC] Test mode reading T/VF...");

            } else {
                // Normal mode: already pulled TE1, wait 2s
                if (elapsed < 2000) break;

                dtcState = DTC_READ_BLINK;
                dtcTimer = now;
                lastTvf = digitalRead(PIN_TVF);
                lastTvfEdge = now;
                readStartTime = now;
                Serial.println("[DTC] Reading T/VF blink pattern...");
            }
            break;
        }

        case DTC_READ_BLINK: {
            runBlinkReader(now);
            break;
        }

        case DTC_SEND_RESULT:
            dtcSendResult();
            dtcState = DTC_CLEANUP;
            dtcTimer = now;
            break;

        case DTC_CLEANUP:
            // Hold pins briefly for cleanup
            dtcCleanup();
            break;
    }
}

// ── Process incoming UDP commands ─────────────────────────────────────
void processUdpCommand() {
    int packetSize = udp.parsePacket();
    if (!packetSize) return;

    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len <= 0) return;
    packetBuffer[len] = '\0';

    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, packetBuffer);
    if (err) {
        Serial.printf("[UDP] Invalid JSON: %s\n", err.c_str());
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    Serial.printf("[UDP] Command: %s\n", cmd);

    if (strcmp(cmd, "read_dtc") == 0) {
        dtcStartRead(false);
    } else if (strcmp(cmd, "read_dtc_test") == 0) {
        dtcStartRead(true);
    } else if (strcmp(cmd, "query") == 0) {
        // Standard query — just continue normal sensor output
    } else if (strcmp(cmd, "serial_tx") == 0) {
        // Forward serial data to HC-05 (or any device on UART2)
        const char* data = doc["data"];
        if (data) {
            Serial2.print(data);
            Serial.printf("[HC05] TX: %s\n", data);
        }
    } else if (strcmp(cmd, "bind") == 0) {
        const char* vin = doc["vin"];
        if (vin && strlen(vin) == 17) {
            binder.bind(vin);
            binder.sendBoundResponse(udp, CARPUTER_IP, SENSOR_PORT);
        } else {
            binder.sendErrorResponse(udp, CARPUTER_IP, SENSOR_PORT, "Invalid VIN");
        }
    } else if (strcmp(cmd, "unbind") == 0) {
        binder.unbind();
        binder.sendUnboundResponse(udp, CARPUTER_IP, SENSOR_PORT);
    }
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    delay(1000);
    Serial.println("Sensor Module Starting (ECU-tap + DTC reader)...");

    pinMode(PIN_MAP, INPUT);
    pinMode(PIN_THW, INPUT);
    pinMode(PIN_VTA, INPUT);
    pinMode(PIN_FUEL, INPUT);
    pinMode(PIN_OIL_PRESS, INPUT);
    pinMode(PIN_O2, INPUT);     // Narrowband O2 — no pullup, 0-1V direct

    pinMode(PIN_SPD, INPUT);
    pinMode(PIN_IGF, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD), speedPulse, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_IGF), igfPulse, RISING);

    pinMode(PIN_DRIVER_DOOR, INPUT_PULLUP);
    pinMode(PIN_PASSENGER_DOOR, INPUT_PULLUP);
    pinMode(PIN_TRUNK, INPUT_PULLUP);
    pinMode(PIN_HOOD, INPUT_PULLUP);

    // DTC pins
    pinMode(PIN_TE1, OUTPUT);
    digitalWrite(PIN_TE1, LOW);  // transistor OFF (TE1 not connected to E1)
    pinMode(PIN_TE2, OUTPUT);
    digitalWrite(PIN_TE2, LOW);  // transistor OFF
    pinMode(PIN_TVF, INPUT);     // T/VF signal from ECU (with voltage divider)

    // HC-05 Bluetooth serial bridge
    Serial2.begin(HC05_BAUD, SERIAL_8N1, PIN_HC05_RX, PIN_HC05_TX);
    pinMode(PIN_HC05_STATE, INPUT);
    Serial.printf("[HC05] UART2 ready on GPIO%d(TX)/%d(RX) at %d baud\n",
                  PIN_HC05_TX, PIN_HC05_RX, HC05_BAUD);

    initSamples();

    WiFi.mode(WIFI_STA);
    WiFi.config(IPAddress(192,168,4,20), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    udp.begin(CMD_PORT);  // Listen on port 5002 for commands, can still send too
    Serial.printf("UDP ready (listening on port %d)\n", CMD_PORT);

    binder.begin(MODULE_SENSOR_MODULE);
    Serial.println("Module binder initialized");
}

// ── Main loop ─────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // Process incoming UDP commands (always)
    processUdpCommand();

    // Run DTC state machine if active
    if (dtcState != DTC_IDLE) {
        runDtcStateMachine(now);
        delay(10);
        return;  // During DTC read, skip normal sensor processing
    }

    // ── HC-05 serial bridge ────────────────────────────────────────────
    // HC-05 STATE pin: HIGH = connected to BT device, LOW = idle
    // Bridge is always active — app can send AT commands or raw data at any time

    // Forward HC-05 serial data → UDP to app
    while (Serial2.available()) {
        char c = Serial2.read();
        if (serialBridgeLen < (int)sizeof(serialBridgeBuf) - 1) {
            serialBridgeBuf[serialBridgeLen++] = c;
        }
        // Send when buffer has data or timeout (50ms)
        if (serialBridgeLen > 0 && (now - lastSerialBridgeSend >= 50)) {
            serialBridgeBuf[serialBridgeLen] = '\0';
            StaticJsonDocument<640> bleDoc;
            bleDoc["event"] = "ble";
            bleDoc["type"] = "serial_rx";
            bleDoc["data"] = serialBridgeBuf;
            udp.beginPacket(CARPUTER_IP, SERIAL_BRIDGE_PORT);
            serializeJson(bleDoc, udp);
            udp.println();
            udp.endPacket();
            serialBridgeLen = 0;
            lastSerialBridgeSend = now;
        }
    }
    // Flush remaining serial data after 50ms of no new data
    if (serialBridgeLen > 0 && (now - lastSerialBridgeSend >= 50)) {
        serialBridgeBuf[serialBridgeLen] = '\0';
        StaticJsonDocument<640> bleDoc;
        bleDoc["event"] = "ble";
        bleDoc["type"] = "serial_rx";
        bleDoc["data"] = serialBridgeBuf;
        udp.beginPacket(CARPUTER_IP, SERIAL_BRIDGE_PORT);
        serializeJson(bleDoc, udp);
        udp.println();
        udp.endPacket();
        serialBridgeLen = 0;
        lastSerialBridgeSend = now;
    }

    // Send HC-05 connection status every 2 seconds
    static unsigned long lastBleStatus = 0;
    if (now - lastBleStatus >= 2000) {
        lastBleStatus = now;
        bool hcConnected = digitalRead(PIN_HC05_STATE);
        StaticJsonDocument<128> bleStat;
        bleStat["event"] = "ble";
        bleStat["type"] = "status";
        bleStat["connected"] = hcConnected;
        udp.beginPacket(CARPUTER_IP, SERIAL_BRIDGE_PORT);
        serializeJson(bleStat, udp);
        udp.println();
        udp.endPacket();
    }

    // Forward app serial commands → HC-05 (handled in processUdpCommand)

    // ── Normal sensor processing ──

    // Check WiFi connection
    if (now - lastWifiCheck >= 5000) {
        lastWifiCheck = now;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected, reconnecting...");
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }

    // Calculate speed (every 100ms)
    unsigned long spdElapsed = now - lastSpeedUpdate;
    if (spdElapsed >= 100) {
        float kmPerHour = (speedPulses * METERS_PER_PULSE * 1000.0) / (spdElapsed / 1000.0);
        currentSpeed = kmPerHour * 0.621371;
        speedPulses = 0;
        lastSpeedUpdate = now;
    }

    // Calculate RPM (every 100ms)
    unsigned long rpmElapsed = now - lastRpmUpdate;
    if (rpmElapsed >= 100) {
        currentRpm = (igfPulses * 1000.0 / rpmElapsed * 60.0) / 2.0;
        igfPulses = 0;
        lastRpmUpdate = now;
    }

    // Read and smooth analog sensors
    float coolant = movingAverage(coolantSamples, ntcFromEcu(analogRead(PIN_THW)));
    float mapVal  = movingAverage(mapSamples, readMapPercent(analogRead(PIN_MAP)));
    float throttle = movingAverage(throttleSamples, readThrottlePercent(analogRead(PIN_VTA)));
    float fuelVal = movingAverage(fuelSamples, readSenderPercent(analogRead(PIN_FUEL), 3.0, 110.0, true));
    float oilPressVal = movingAverage(oilPressSamples, readSenderPercent(analogRead(PIN_OIL_PRESS), 3.0, 100.0, false));
    float o2Afr = movingAverage(o2Samples, o2ToAfr(analogRead(PIN_O2)));
    sampleIndex++;

    // Read door sensors
    bool driverDoor = !digitalRead(PIN_DRIVER_DOOR);
    bool passengerDoor = !digitalRead(PIN_PASSENGER_DOOR);
    bool trunk = !digitalRead(PIN_TRUNK);
    bool hood = !digitalRead(PIN_HOOD);

    // Send sensors every 500ms
    if (now - lastUpdate >= 500) {
        lastUpdate = now;

        StaticJsonDocument<256> doc;
        doc["event"] = "sensors";
        JsonObject data = doc.createNestedObject("data");

        data["speed"]      = (int)currentSpeed;
        data["rpm"]        = (int)currentRpm;
        data["coolant"]    = (int)coolant;
        data["throttle"]   = (int)throttle;
        data["map"]        = (int)mapVal;
        data["fuel"]       = (int)fuelVal;
        data["oilPressure"] = (int)oilPressVal;
        data["o2AFR"]       = (int)(o2Afr * 10) / 10.0;  // 1 decimal place
        data["driverDoor"] = driverDoor;
        data["passengerDoor"] = passengerDoor;
        data["trunk"]      = trunk;
        data["hood"]       = hood;

        // OBD2 data (populated by app via serial bridge, echoed here for convenience)
        data["obd2Rpm"]      = obd2Rpm;
        data["obd2Speed"]    = obd2Speed;
        data["obd2Load"]     = obd2Load;
        data["obd2Coolant"]  = obd2Coolant;
        data["obd2Intake"]   = obd2Intake;
        data["obd2Timing"]   = obd2Timing;
        data["obd2FuelTrimS"]= obd2FuelTrimS;
        data["obd2FuelTrimL"]= obd2FuelTrimL;
        data["obd2FuelPress"]= obd2FuelPress;
        data["obd2DistDtc"]  = obd2DistDtc;

        // Broadcast so both carputer app and cluster display receive data
        IPAddress broadcastIp = WiFi.localIP();
        broadcastIp[3] = 255;  // Subnet broadcast
        udp.beginPacket(broadcastIp, SENSOR_PORT);
        serializeJson(doc, udp);
        udp.println();
        udp.endPacket();
    }

    delay(10);
}
