# Speeduino to CAN Interface for BMW E39/E46 Instrument Clusters

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This project reads real-time data from Speeduino EFI system via UART and converts it to BMW-specific CAN messages for E39/E46 instrument clusters.

## Parts
- 1x ESP32-WROOM-32D - <img width="727" height="508" alt="image" src="https://github.com/user-attachments/assets/15ae8512-49b9-4f0c-8aec-2b2c67a8fb51" />
- 1x MCP2551 - <img width="1000" height="1000" alt="image" src="https://github.com/user-attachments/assets/cb113db2-60f7-4216-8dc7-466d4a989312" />
- 1x logic level converter 3.3V to 5V <img width="877" height="770" alt="image" src="https://github.com/user-attachments/assets/0d80677b-53cd-4988-9236-8510f23269a1" />
- or create voltage divider <img width="408" height="151" alt="image" src="https://github.com/user-attachments/assets/4e58a212-3535-470a-83a7-41f3adcdf146" />
- probably also u need Step-down converter to powerup esp32 <img width="800" height="800" alt="image" src="https://github.com/user-attachments/assets/f336b0d8-da8c-4496-a9a9-f8525744cfa7" />


## Features
- Real-time RPM, coolant temp, TPS, fuel consumption and other data from speeduino
- Plug-and-play with Speeduino via Serial (115200 baud)
- CAN bus communication @ 500kbps
- Future updates and full compatibility with my future add-ons

## Installation
1. Use Arduino IDE to flash ESP32
2. Connect:
   - Serial RX/TX pins from ur config wia logic level converter/voltage divider to arduino mega/speeduino pin of serial3 (tx3 - 14 /rx3 - 15) - <img width="2571" height="2572" alt="image" src="https://github.com/user-attachments/assets/038b7550-2ef1-4453-8c64-1f61b977c0ef" />
   - Connect esp32 CAN_RX/TX pins to MCP2551
   - CAN H/L from to vehicle CAN bus
   - Get powersupply to all modules - esp,mcp
3. Get fun of ur speeduino project :D

```c++
// Example connection:
#define SERIAL2_RX_PIN 16   // ESP32 UART2 RX
#define SERIAL2_TX_PIN 17   // ESP32 UART2 TX
#define CAN_RX_PIN     4    // MCP2551 RX
#define CAN_TX_PIN     5    // MCP2551 TX
