#include "uart_comm.h"
#include "config.h"

#include <Arduino.h>
#include <ArduinoJson.h>

void uart_comm_init(unsigned long baud_rate)
{
    Serial.begin(baud_rate);
    while (!Serial) { /* wait for USB CDC on boards that need it */ }
}

void uart_comm_send(const SensorData &data)
{
    // Fixed-capacity JSON document (stack-allocated, no heap fragmentation)
    JsonDocument doc;

    doc["map_kpa"]    = serialized(String(data.map_kpa,   2));
    doc["tps_pct"]    = serialized(String(data.tps_pct,   2));
    doc["cts_degc"]   = serialized(String(data.cts_degc,  2));
    doc["iat_degc"]   = serialized(String(data.iat_degc,  2));
    doc["o2_lambda"]  = serialized(String(data.o2_lambda, 3));
    doc["batt_v"]     = serialized(String(data.batt_v,    2));
    doc["valid"]      = data.valid;

    serializeJson(doc, Serial);
    Serial.println();   // newline delimiter for line-based parsers
}
