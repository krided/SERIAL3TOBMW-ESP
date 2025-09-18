# Speeduino to CAN Interface for BMW E39/E46 Instrument Clusters

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[![Buy Me A Coffee](https://img.shields.io/badge/Buy%20Me%20a%20Coffee-ffdd00?&logo=buy-me-a-coffee&logoColor=black)](https://buycoffee.to/krided)

This project reads real-time data from the **Speeduino EFI system** via UART and converts it into BMW-specific CAN messages for **E39/E46 instrument clusters**. It’s designed for ESP32 WROOM32D and MCP2551 transceiver, with optional web/Bluetooth dashboard support.
---

## VERSIONS!

**BASIC**
- Basic code like, exacly one to one features from pazi88 code but on esp32 with mcp2515
- Added blinking hot led 
- Will have only small updates for stability. Nothink more
- Work with **1x MCP2515 CAN Transceiver**

**MAIN**
- STILL ON WORK!
- Main code what will be used. 
- Will have all features and updates.
- Work with **1x MCP2551 CAN Transceiver**
---

## Features

- Plug-and-play connection to Speeduino via **Serial3** (115200 baud).
- CAN communication at **500 kbps** for BMW E39/E46 clusters.
- Optional Bluetooth dashboard for phone or tablet.

- Configurable via **central `config.h`** file:
  - BTE credentials
  - Dashbord mapping. **In progress**
  - CAN & UART settings.
  - Dashboard variables.
  - Update rates.
  - Temperature waring.
-
---

## In progress

- Full custom real-time monitoring: RPM, coolant temp, TPS, fuel level, oil temp, ambient temp, AFR, and more. via site.
- More warings like the temperature.
- Wireless inputs to speeduino (Reving up, etc.)

---

## Parts Required

- **1x ESP32-WROOM-32D**  
  <img width="727" height="508" src="https://github.com/user-attachments/assets/15ae8512-49b9-4f0c-8aec-2b2c67a8fb51" />

- **1x MCP2551 CAN Transceiver** / **1x MCP2515 CAN Transceiver**
  <img width="1000" height="1000" src="https://github.com/user-attachments/assets/cb113db2-60f7-4216-8dc7-466d4a989312" />

- **1x Logic Level Converter (3.3V ↔ 5V)**  
  <img width="877" height="770" src="https://github.com/user-attachments/assets/0d80677b-53cd-4988-9236-8510f23269a1" />  
  And/or use 2x a simple **voltage divider**:  
  <img width="408" height="151" src="https://github.com/user-attachments/assets/4e58a212-3535-470a-83a7-41f3adcdf146" />
---

## Installation

1. Install **Arduino IDE** and ESP32 board support: [Arduino ESP32 setup guide](https://github.com/espressif/arduino-esp32)
2. Clone this repository.
3. Open `config.h` and configure:
   - **WiFi credentials** (if using dashboard)
   - **CAN RX/TX** pins (ESP32 → MCP2551)
   - **UART RX/TX** pins (ESP32 → Speeduino Serial3)
   - **Variables to monitor** in the dashboard (e.g., RPM, AFR)
4. Connect your hardware:
   - ESP32 UART ↔ Speeduino Serial3 (via logic level converter)
   - ESP32 CAN ↔ MCP2551 ↔ Vehicle CAN bus
   - Power ESP32 and MCP2551 from 5V pin from speeduino (ESP32 VIN pin)
5. Upload code via Arduino IDE.
6. Enjoy monitoring your Speeduino data on BMW cluster or optional dashboard.

Dont forget to have one ground with MCP2551 ESP32 and Speeduino!
CTX pin from MCP2551 and TX pin from Speeduino MUST HAVE have voltage divider. For best, fastest and cheapest solution get voltage divider. With cheap level converters sometimes u can get errors from can/arduino or even code will be not work!
I recomend using logic level converter only to ESP32 <-> ARDUINO MEGA not for MCP2551

---

## Example Connection

```c++
// ==================== PINS ====================
#define SERIAL1_RX_PIN 22 // GPIO22 (Serial1 RX)
#define SERIAL1_TX_PIN 23 // GPIO23 (Serial1 TX)
#define CAN_RX_PIN     4  // GPIO4 (TWAI/CAN RX) - MCP2551
#define CAN_TX_PIN     5  // GPIO5 (TWAI/CAN TX) - MCP2551
// ==================== Serial settings ====================
#define SERIAL_BAUDRATE 115200  // Baudrate for Serial2 communication with Speeduino
#define SERIAL_DEBUG_BAUDRATE 115200 // Baudrate for Serial debugging
#define CAN_BAUDRATE    500E3  // CAN bus speed
// ==================== TEMPERATURE LIGHT ====================
#define HOT_TEMP_BLINK 100    // Set temperature for overheat light to start blinking
#define HOT_TEMP_SOLID 120    // Set temperature for overheat light to stay solid
#define BLINKING 1            // 1 = enable blinking, 0 = solid light only
// ==================== UPDATE RATE ====================
#define SerialUpdateRate 30   // 30 Hz rate limit to update secondary serial data from speeduino
#define ClusterUpdateRate 50  // 50 Hz Frequency for the cars instrument cluster
```

---

## Dashboard & Config

All variables you want to monitor are defined in **`config.h`**.  

Advantages:

- Add or remove variables without touching `main.ino`.
- Define update rate to reduce CPU load.
- Centralized Bluetooth settings for optional mobile dashboard.

---

## Contribution & Credits

This project is heavily inspired by the work of **pazi88** in the community. Parts of the codebase are derived from existing open-source projects, adapted for ESP32 and BMW-specific CAN messaging. The author has rewritten and reorganized the logic, added dashboard support, configuration flexibility, and full documentation for hobbyist and professional use.

---

## License

MIT License – see [LICENSE](LICENSE)

