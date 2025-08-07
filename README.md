# Speeduino to CAN Interface for BMW E39/E46 Instrument Clusters

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This project reads real-time data from Speeduino EFI system via UART and converts it to BMW-specific CAN messages for E39/E46 instrument clusters.

## Features
- Real-time RPM, coolant temp, TPS, fuel consumption data
- Plug-and-play with Speeduino via Serial (115200 baud)
- CAN bus communication @ 500kbps
- ESP32 WROOM32D & MCP2551 hardware support

## Installation
1. Use Arduino IDE to flash ESP32
2. Connect:
   - Serial RX/TX to Speeduino
   - CAN H/L to vehicle CAN bus
3. Configure Speeduino to output realtime data

```c++
// Example connection:
#define SERIAL2_RX_PIN 16   // ESP32 UART2 RX
#define SERIAL2_TX_PIN 17   // ESP32 UART2 TX
#define CAN_RX_PIN     4    // MCP2551 RX
#define CAN_TX_PIN     5    // MCP2551 TX