#pragma once
#include <Arduino.h>

struct Telemetry {
    uint16_t RPM = 0;
    uint16_t PW = 0;
    uint8_t CLT = 0;
    uint8_t TPS = 0;
    uint8_t AFR = 0;
    uint8_t FuelLevel = 0;
    uint8_t tempLight = 0;
    uint8_t boostDuty = 0;
    uint8_t ethanolPct = 0;
    // here adding more telemetry fields as needed
};

extern Telemetry telemetry;

String telemetryToJson(); // function to convert telemetry data to JSON
