#include "telemetry.h"

Telemetry telemetry;

String telemetryToJson() {
    String json = "{";
    json += "\"RPM\":" + String(telemetry.RPM) + ",";
    json += "\"PW\":" + String(telemetry.PW) + ",";
    json += "\"CLT\":" + String(telemetry.CLT) + ",";
    json += "\"TPS\":" + String(telemetry.TPS) + ",";
    json += "\"AFR\":" + String(telemetry.AFR) + ",";
    json += "\"FuelLevel\":" + String(telemetry.FuelLevel) + ",";
    json += "\"tempLight\":" + String(telemetry.tempLight) + ",";
    json += "\"boostDuty\":" + String(telemetry.boostDuty) + ",";
    json += "\"ethanolPct\":" + String(telemetry.ethanolPct);
    json += "}";
    return json;
}
    