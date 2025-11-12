# Speeduino to CAN Interface for BMW E39/E46 Instrument Clusters

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[![Buy Me A Coffee](https://img.shields.io/badge/Buy%20Me%20a%20Coffee-ffdd00?&logo=buy-me-a-coffee&logoColor=black)](https://buycoffee.to/krided)

This project reads real-time data from the **Speeduino EFI system** via UART and converts it into BMW-specific CAN messages for **E39/E46 instrument clusters**. It is designed for the ESP32 WROOM32D and an MCP2515 CAN controller/transceiver, and includes optional web/Bluetooth dashboard support.

---

## Basic

- Basic code: essentially a one-to-one port of pazi88's implementation, but adapted for ESP32 with an MCP2515.
- Added a blinking "hot" LED indicator.
- The project will receive only small stability updates; no major new features are planned.
- Works with a single **MCP2515 CAN transceiver**.


## Features

- Plug-and-play connection to Speeduino via **Serial3** (115200 baud).
- CAN communication at **500 kbps** for BMW E39/E46 clusters.
- Optional Bluetooth dashboard for phone or tablet.

- Configurable via the central **`config.h`** file:
  - Bluetooth (BT) credentials.
  - Dashboard mapping (in progress).
  - CAN & UART settings.
  - Dashboard variables.
  - Update rates.
  - Temperature warnings.

---

## In progress

- Development of full custom real-time monitoring: RPM, coolant temperature, TPS, fuel level, oil temperature, ambient temperature, AFR, and more (via the dashboard/site).
- Additional warnings (e.g., temperature alerts).
- Wireless inputs to Speeduino (e.g., rev detection).

---

## Parts required

- **ESP32-WROOM-32D**
  <img width="727" height="508" src="https://github.com/user-attachments/assets/15ae8512-49b9-4f0c-8aec-2b2c67a8fb51" />

- **MCP2515 CAN transceiver**

- **Logic level converter (3.3V ↔ 5V)**
  <img width="877" height="770" src="https://github.com/user-attachments/assets/0d80677b-53cd-4988-9236-8510f23269a1" />
  Alternatively, a simple **voltage divider** can be used:
  <img width="408" height="151" src="https://github.com/user-attachments/assets/4e58a212-3535-470a-83a7-41f3adcdf146" />

---

## Installation

1. Install the **Arduino IDE** and ESP32 board support: [Arduino ESP32 setup guide](https://github.com/espressif/arduino-esp32).
2. Clone this repository.
3. Connect your hardware:
   - ESP32 UART ↔ logic level converter ↔ Speeduino Serial3.
   - ESP32 SPI ↔ MCP2515.
   - MCP2515 ↔ CAN
   - Power the ESP32 (VIN pin) and the MCP2515.
4. Optional: change personal options in `config.h`.
5. Copy the contents of the `Libs/` folder into your Arduino IDE libraries folder.
6. Upload the code via the Arduino IDE.
7. Enjoy your working instrument cluster.

Don't forget to share a common ground between the MCP2515, ESP32, and Speeduino.

The MCP2515 CTX pin and the Speeduino TX pin must be interfaced using a proper voltage divider (or a reliable level shifter). Cheap level converters can sometimes cause CAN/Arduino errors or prevent the code from working correctly.

---

## Example connection

```c++
#define SERIAL1_RX_PIN 22 // GPIO22 (Serial1 RX)
#define SERIAL1_TX_PIN 23 // GPIO23 (Serial1 TX)
// =======================================================

// =================== MCP2515 SPI pins ===================
#define MCP2515_CS_PIN   5  // GPIO5 (SPI CS)
#define MCP2515_INT_PIN  4  // GPIO4 (MCP2515 INT)
#define MCP2515_MOSI_PIN 21 // GPIO21 (SPI MOSI) - custom pin to avoid conflict with Serial1
#define MCP2515_MISO_PIN 19 // GPIO19 (SPI MISO)
#define MCP2515_SCK_PIN  18 // GPIO18 (SPI SCK)
```

---

## Dashboard & config

All variables you want to monitor are defined in **`config.h`**.

---

## ANY PROBLEMS

If u have any problems with upload project u also have .bin to upload on bin folder. 
U find tutorials for it online.

---

## Contribution & credits

This project is heavily inspired by the work of **pazi88**. Parts of the codebase are derived from existing open-source projects and adapted for ESP32 and BMW-specific CAN messaging. The author rewrote and reorganized the logic, added dashboard support, improved configuration flexibility, and created documentation for hobbyists and professionals.

---

## License

MIT License – see [LICENSE](LICENSE)

