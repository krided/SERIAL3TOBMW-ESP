# MCP2515 Wiring Guide for ESP32

## Hardware Requirements
- ESP32 WROOM32D development board
- MCP2515 CAN controller module (usually includes MCP2551 transceiver)
- BMW E39/E46 vehicle with CAN bus access

## Wiring Connections

### ESP32 to MCP2515 Module
| ESP32 Pin | MCP2515 Pin | Function |
|-----------|-------------|----------|
| GPIO5     | CS          | Chip Select |
| GPIO4     | INT         | Interrupt |
| GPIO21    | MOSI/SI     | SPI Data Out |
| GPIO19    | MISO/SO     | SPI Data In |
| GPIO18    | SCK/CLK     | SPI Clock |
| 3.3V      | VCC         | Power Supply |
| GND       | GND         | Ground |

### Speeduino Connection
| ESP32 Pin | Speeduino Pin | Function |
|-----------|---------------|----------|
| GPIO22    | Serial3 TX    | Data from Speeduino |
| GPIO23    | Serial3 RX    | Data to Speeduino |
| GND       | GND           | Ground |

### MCP2515 to BMW CAN Bus
Connect the CAN H and CAN L wires from the MCP2515 module to the BMW's CAN bus wires.

**Warning:** Always double-check your vehicle's CAN bus wiring before connecting!

## Notes
- Most MCP2515 modules use an 8MHz crystal oscillator
- If your module uses a 16MHz crystal, uncomment the appropriate line in the setup() function
- Ensure proper power supply (3.3V) to avoid damaging the ESP32
- Use proper CAN bus termination resistors if required
