#pragma once

#include "sensors.h"

// =============================================================================
// UART communication interface
// Serialises SensorData to a JSON line and writes it to the hardware Serial
// port (UART0).  No CAN bus is used.
// =============================================================================

// Initialise the Serial port.  Call once from setup().
void uart_comm_init(unsigned long baud_rate);

// Transmit one JSON line containing the full sensor snapshot.
// Format (newline-terminated, fields in engineering units):
//   {"map_kpa":95.2,"tps_pct":12.5,"cts_degc":85.1,"iat_degc":32.4,
//    "o2_lambda":1.01,"batt_v":13.8,"valid":true}
void uart_comm_send(const SensorData &data);
