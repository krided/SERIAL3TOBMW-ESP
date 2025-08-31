#pragma once

// ==================== PINS ====================
#define SERIAL2_RX_PIN 16 // GPIO16 (RX2)
#define SERIAL2_TX_PIN 17 // GPIO17 (TX2)
#define CAN_RX_PIN     4 // GPIO4 (CAN RX)
#define CAN_TX_PIN     5 // GPIO5 (CAN TX)
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

// ==================== FUEL / INJECTOR ====================
#define PW_ADJUST 19000           // scaling for fuel consumption calculation

// ====================== BT ======================
#define BT_NAME       "ESP32_Dash"  // Name of the BT device
#define BT_PASSWORD   "1234"        // Password for the BT device (only if needed, usually not)
#define DASH_UPLOAD_MS 750          // how often to send data
#define DASH_UPDATE_MS 500          // how often to update data from telemetry
// ====================== Dashboard =====================
#define MAX_DASH_VALUES 20          // max number of variables on the dashboard

// List of variables to send (add your names here)
inline  const char* DASH_KEYS[MAX_DASH_VALUES] = {
   // "rpm",
   // "afr",
   // "oiltemp",
   // "tps"
};

// Table of start the actual values
inline  float DASH_VALUES[MAX_DASH_VALUES];
// ==================== DEBUG ====================
#define DEBUG_SERIAL 1            // 1 = enable logs in Serial, 0 = disable

// Updating dash values
inline void updateDashValues() {
    //dash,set_NAMEOF_YOUR_VARIABLE(Variable from currentStatus);

    //dash.setRPM(currentStatus.RPM);

    //dash.update(); // Update the dashboard
  }
//================================