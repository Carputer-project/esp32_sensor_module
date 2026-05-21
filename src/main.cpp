#include <Arduino.h>

#include "config.h"
#include "sensors.h"
#include "uart_comm.h"

// =============================================================================
// ESP32 Analog Sensor Module for Car ECU
// Reads MAP, TPS, CTS, IAT, O2, and battery-voltage sensors via the on-chip
// ADC and streams JSON data over UART at UPDATE_RATE_MS intervals.
// No CAN bus is required.
// =============================================================================

void setup()
{
    uart_comm_init(UART_BAUD_RATE);
    sensors_init();
}

void loop()
{
    static uint32_t last_ms = 0;
    uint32_t now = millis();

    if (now - last_ms >= UPDATE_RATE_MS) {
        last_ms = now;
        SensorData data = sensors_read();
        uart_comm_send(data);
    }
}
