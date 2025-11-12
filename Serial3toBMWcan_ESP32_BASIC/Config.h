// ============ Pin definitions for ESP32 ================
#define SERIAL1_RX_PIN 22 // GPIO22 (Serial1 RX)
#define SERIAL1_TX_PIN 23 // GPIO23 (Serial1 TX)
// =======================================================

// =================== MCP2515 SPI pins ===================
#define MCP2515_CS_PIN   5  // GPIO5 (SPI CS)
#define MCP2515_INT_PIN  4  // GPIO4 (MCP2515 INT)
#define MCP2515_MOSI_PIN 21 // GPIO21 (SPI MOSI) - custom pin to avoid conflict with Serial1
#define MCP2515_MISO_PIN 19 // GPIO19 (SPI MISO)
#define MCP2515_SCK_PIN  18 // GPIO18 (SPI SCK)
// =======================================================

// =================== INJECTOR SCALING ==================
// Just write the flow rate of your injectors here in CC/min
// Used to scale the pulsewidth values sent to the cluster

#define FLOW_CC 250 // Write here your injector flow rate in CC/min

// BASE values for 250cc injectors - do not change these!
#define BASE_FLOW 250     // base injector (M52TU 250cc)
#define BASE_PW   25000   // PW_ADJUST for 250cc
//======================================================

// ==================== TEMPERATURE LIGHT =================
#define HOT_TEMP_BLINK 110    // Set temperature for overheat light to start blinking
#define HOT_TEMP_SOLID 130    // Set temperature for overheat light to stay solid
#define BLINKING 1            // 1 = enable blinking, 0 = solid light only
// ========================================================

// ==================== DEBUG OPTIONS ====================
#define DEBUG 0  // Set to 1 to enable debug messages via Serial
// =======================================================

// ======= UPDATE RATES in Hz and related settings =======
#define SerialUpdateRate 30   // 30 Hz rate limit to update secondary serial data from speeduino
#define PWMFanFrequency 100   // 100 Hz Frequency for the BMW PWM fan
#define ClusterUpdateRate 50  // 50 Hz Frequency for the cars instrument cluster
// =======================================================

// ================ AIR CONDITIONING ====================== !!! IN PROGRESS !!!
#define BIT_LV_REQ_TCO_L       5  // Request For Lowering Cooling Temp (c_tco_bol_ect)
#define BIT_LV_ACCIN           6  // Air Conditioning Compressor Status (0=off, 1=on)
#define BIT_LV_ACIN            7  // Air Conditioning Request (0=off, 1=on)
// ========================================================

// ========== Bitsetting macros and state defines ==========
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))
#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define A_MESSAGE               2
#define PWM_MESSAGE             3
// ========================================================