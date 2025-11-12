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

## Install and config

**On folders BASIC and MAIN u will get readme.md for version what u looking where u find all info.**

## Contribution & Credits

This project is heavily inspired by the work of **pazi88** in the community. Parts of the codebase are derived from existing open-source projects, adapted for ESP32 and BMW-specific CAN messaging. The author has rewritten and reorganized the logic, added dashboard support, configuration flexibility, and full documentation for hobbyist and professional use.

---

## License

MIT License – see [LICENSE](LICENSE)

