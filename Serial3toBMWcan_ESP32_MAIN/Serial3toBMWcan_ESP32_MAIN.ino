/*
  BMW E39/E46 CAN Bridge & Dashboard for ESP32 WROOM32D + MCP2515
  ----------------------------------------------------------------

  This project is an ESP32-based solution that reads real-time data from
  a Speeduino EFI system via Serial3 and converts it into CAN messages 
  compatible with BMW E39/E46 instrument clusters. It also provides a
  customizable dashboard, supporting Bluetooth telemetry for mobile devices.

  Hardware requirements:
  - ESP32 WROOM32D
  - MCP2515 CAN controller module
  - MCP2551 CAN transceiver (usually included with MCP2515 modules)

  Portions of this code are derived from work by the user "pazi88", who
  originally developed the Speeduino-to-CAN BMW bridge for STM32. Their contributions
  have been adapted and extended in this project, including new features such as:
    - Bluetooth data streaming
    - Configurable dashboard items
    - Flexible data mapping for engine parameters
    - Optimized update rates to reduce unnecessary load
    - And a lot more improvements

  This project is provided "as-is" and comes without any warranty. The author
  assumes no responsibility for any damage, misbehavior, or data inaccuracies
  resulting from its use.

  Note: This is a collaborative adaptation. While acknowledging the original
  work, substantial modifications and enhancements have been implemented
  to fit ESP32 hardware and additional functionality.
*/

/*
Libs used:
[CAN@0.3.1] - Arduino CAN library with MCP2515 support
[BluetoothSerial@3.3.0] - Bluetooth Serial library for ESP32
*/

// ========== Library ===========
#include "config.h" // configuration file
#include <Arduino.h>
#include <SPI.h>
#include <CAN.h>
#include "dashboard.h" // for wifi dashboard
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
// ===============================

// ====== GLOBAL VARIABLES ======
Dashboard dash;
// ==========================================

// ====== BIT MANIPULATION MACROS ======
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : BIT_CLEAR((var), (pos)))

// ====== SERIAL MESSAGE TYPES ======
#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define A_MESSAGE               2
#define PWM_MESSAGE             3
// ===============================

// ====== CAN MESSAGE STRUCTURES ======
// Struct for CAN messages
struct CAN_message_t {
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
};
static CAN_message_t CAN_msg_RPM;     // RPM and torque data
static CAN_message_t CAN_msg_CLT_TPS; // Coolant temp and TPS data
static CAN_message_t CAN_msg_MPG_CEL; // Fuel consumption and error lights
static CAN_message_t CAN_inMsg;       // Incoming CAN messages
// ===============================

// ====== DASHBOARD VARIABLES ======
unsigned long lastDashUpdate = 0; // last time dashboard was updated
const unsigned long dashupdaterate = DASH_UPLOAD_MS - DASH_UPDATE_MS; // dashboard update rate from config.h

// ====== SPEEDUINO DATA STRUCTURE ======
// This struct gathers data read from Speeduino. This is a direct copy of Speeduino's internal structure
struct statuses {
  uint8_t secl; // secl is simply a counter that increments each second.
  uint8_t status1; // status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
  uint8_t engine; // Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
  uint8_t dwell; // Dwell in ms * 10
  uint16_t MAP; // 2 bytes for MAP
  uint8_t IAT;
  uint8_t CLT;
  uint8_t batCorrection; // Battery voltage correction (%)
  uint8_t battery10; // battery voltage
  uint8_t O2; // O2
  uint8_t egoCorrection; // Exhaust gas correction (%)
  uint8_t iatCorrection; // Air temperature Correction (%)
  uint8_t wueCorrection; // Warmup enrichment (%)
  uint16_t RPM; // rpm
  uint8_t AEamount; // acceleration enrichment (%)
  uint8_t corrections; // Total GammaE (%)
  uint8_t VE; // Current VE 1 (%)
  uint8_t afrTarget;
  uint16_t PW1; // Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  uint8_t tpsDOT; // TPS DOT
  int8_t advance;
  uint8_t TPS; // TPS (0% to 100%)
  uint16_t loopsPerSecond;
  uint16_t freeRAM;
  uint8_t boostTarget; // boost target divided by 2 to fit in a byte
  uint8_t boostDuty;
  uint8_t spark; // Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
  uint16_t rpmDOT;
  uint8_t ethanolPct; // Flex sensor value (or 0 if not used)
  uint8_t flexCorrection; // Flex fuel correction (% above or below 100)
  uint8_t flexIgnCorrection; // Ignition correction (Increased degrees of advance) for flex fuel
  uint8_t idleLoad;
  uint8_t testOutputs; // testEnabled(0), testActive(1)
  uint8_t O2_2; // O2
  uint8_t baro; // Barometer value
  uint16_t CANin_1;
  uint16_t CANin_2;
  uint16_t CANin_3;
  uint16_t CANin_4;
  uint16_t CANin_5;
  uint16_t CANin_6;
  uint16_t CANin_7;
  uint16_t CANin_8;
  uint16_t CANin_9;
  uint16_t CANin_10;
  uint16_t CANin_11;
  uint16_t CANin_12;
  uint16_t CANin_13;
  uint16_t CANin_14;
  uint16_t CANin_15;
  uint16_t CANin_16;
  uint8_t tpsADC;
  uint8_t getNextError;
  uint8_t launchCorrection;
};

statuses currentStatus;

// ====== COMMUNICATION VARIABLES ======
static uint32_t oldtime = millis();   // for the timeout detection
uint8_t SpeedyResponse[100]; // Data buffer for serial communication with Speeduino
uint8_t rpmLSB;   // Least significant byte for RPM message
uint8_t rpmMSB;   // Most significant byte for RPM message
uint8_t pwLSB;    // Least significant byte for PW message
uint8_t pwMSB;    // Most significant byte for PW message
uint8_t CEL;      // Timer for how long CEL light should be kept on
uint32_t updatePW;
uint8_t odometerLSB;
uint8_t odometerMSB;
uint8_t FuelLevel;
uint8_t ambientTemp;
int CLT; // Coolant temperature for BMW cluster
uint32_t PWcount;
uint8_t TPS, tempLight; // TPS value and overheat light status
bool data_error; // Indicator for data validity from Speeduino
bool responseSent; // Track if we have responded to data request
bool newData; // Indicates if we have new data available from Speeduino
bool ascMSG; // ASC message received status
uint8_t rRequestCounter; // Request PWM fan duty from Speeduino once per second
uint8_t PWMfanDuty; // PWM fan duty cycle
uint8_t data[255]; // For DS2 data
uint8_t SerialState, canin_channel, currentCommand;
uint16_t CanAddress, runningClock;
uint16_t VSS, VSS1, VSS2, VSS3, VSS4; // Vehicle speed sensor values
uint8_t MSGcounter; // Tracks which multiplexed info is sent in 0x329 byte 0
uint8_t multiplex;
uint8_t radOutletTemp;
uint8_t oilTemp;
uint8_t acBitfield;
uint8_t acBitfield2;
uint8_t eFanBitfield;
bool doRequest = true;
// ===============================

// ====== TIMEOUT AND RATES ======
const uint16_t SPEEDUINO_TIMEOUT_MS = 500;  // Timeout for Speeduino communication
const uint8_t RPM_CONVERSION_FACTOR = 6.4;  // RPM conversion factor for BMW cluster
const uint8_t CLT_TEMP_OFFSET = 40;          // CLT temperature offset
const uint16_t MAX_ENGINE_RPM = 8000;        // Maximum expected engine RPM
const uint8_t MAX_CLT_TEMP = 182;            // Maximum CLT temperature (142°C)
const uint8_t MAX_TPS_VALUE = 200;           // Maximum TPS value
// ===============================

// =========== DEBUG =============
#if DEBUG_SERIAL
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)   // nic
  #define DEBUG_PRINTLN(x) // nic
#endif
// ===============================

// ====== FREERTOS TASK FUNCTIONS ======

/**
 * Main task for handling Speeduino communication and dashboard updates
 * This task runs continuously and handles:
 * - Serial data requests to Speeduino every 33ms (30Hz)
 * - Processing incoming serial messages (A and R type)
 * - Updating dashboard values at configured rate
 * - Managing communication timeouts (500ms)
 * - Reading incoming CAN messages
 */
void mainTask(void *pvParameters) {
  (void) pvParameters;
  
  Serial.println("MainTask started");
  uint32_t loopCounter = 0;
  uint32_t lastHeartbeat = millis();
  
  for(;;) {
    loopCounter++;
    
    // Heartbeat every 30 seconds (reduced from 5s to minimize spam)
    if (millis() - lastHeartbeat > 30000) {
      Serial.printf("MainTask heartbeat - loops: %lu, free heap: %u\n", loopCounter, ESP.getFreeHeap());
      lastHeartbeat = millis();
      loopCounter = 0;
    }
    
    if (doRequest) {
      requestData();
    }
    displayData();
    unsigned long NowDashUpdate = millis();
    
    switch(SerialState) {
      case NOTHING_RECEIVED:
        if (Serial1.available() > 0) { 
          ReadSerial(); 
        }
        break;
      case A_MESSAGE:
        if (Serial1.available() >= 74) { 
          HandleA(); 
        } else {
          // Timeout protection for incomplete A messages
          static uint32_t aMessageStartTime = 0;
          if (aMessageStartTime == 0) {
            aMessageStartTime = millis();
          } else if (millis() - aMessageStartTime > 100) { // 100ms timeout
            SerialState = NOTHING_RECEIVED;
            aMessageStartTime = 0;
            Serial.println("A message timeout - resetting state");
          }
        }
        break;
      case R_MESSAGE:
        if (Serial1.available() >= 3) {  
          HandleR(); 
        } else {
          // Timeout protection for incomplete R messages
          static uint32_t rMessageStartTime = 0;
          if (rMessageStartTime == 0) {
            rMessageStartTime = millis();
          } else if (millis() - rMessageStartTime > 100) { // 100ms timeout
            SerialState = NOTHING_RECEIVED;
            rMessageStartTime = 0;
            Serial.println("R message timeout - resetting state");
          }
        }
        break;
      case PWM_MESSAGE:
        // Not implemented, add if needed
        break;
      default:
        SerialState = NOTHING_RECEIVED; // Reset unknown states
        break;
    }
    if ((millis() - oldtime) > SPEEDUINO_TIMEOUT_MS) {
      oldtime = millis();
      static uint32_t timeoutCount = 0;
      timeoutCount++;
      
      Serial.printf("Timeout from speeduino! Count: %lu\n", timeoutCount);
      
      // If we have too many timeouts, try to reset communication
      if (timeoutCount % 10 == 0) { // Every 10th timeout
        Serial.println("Multiple timeouts detected - resetting Serial1 communication");
        
        // Flush both buffers
        while (Serial1.available()) Serial1.read();
        Serial1.flush();
        
        // Reset state
        SerialState = NOTHING_RECEIVED;
        
        // Small delay before requesting again
        delay(50);
      }
      
      doRequest = true;
    }
    
    readCanMessage();
    
    // Update dash values at defined rate 
    if (NowDashUpdate - lastDashUpdate > dashupdaterate) {
      lastDashUpdate = NowDashUpdate;
      updateDashValues(); // Function defined in config.h
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay to prevent CPU hogging
  }
}

/**
 * CAN task for sending messages to BMW cluster at 50Hz
 * This ensures smooth operation of the BMW instrument cluster
 */
void canTask(void *pvParameters) {
  (void) pvParameters;
  
  Serial.println("CANTask started");
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t canLoopCounter = 0;
  uint32_t lastCanHeartbeat = millis();
  
  for(;;) {
    canLoopCounter++;
    
    // Heartbeat every 60 seconds for CAN task
    if (millis() - lastCanHeartbeat > 60000) {
      Serial.printf("CANTask heartbeat - loops: %lu\n", canLoopCounter);
      lastCanHeartbeat = millis();
      canLoopCounter = 0;
    }
    
    SendData(); // Send CAN messages at 50Hz
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ====== SPEEDUINO COMMUNICATION FUNCTIONS ======

// Request data from Speeduino at defined rate
void requestData() {
  static uint32_t lastRequestTime = 0;
  const uint32_t MIN_REQUEST_INTERVAL = 30; // Minimum 30ms between requests (33Hz)
  
  if (doRequest == true && (millis() - lastRequestTime >= MIN_REQUEST_INTERVAL)){
    // Check if Serial1 is ready
    if (!Serial1) {
      Serial.println("Serial1 not ready for request");
      return;
    }
    
    Serial1.write("A"); // Send A to request real time data
    Serial1.flush(); // Ensure the byte is sent immediately
    doRequest = false;
    lastRequestTime = millis();
    
    DEBUG_PRINTLN("Sent A request to Speeduino");
  }
}

// ====== CAN MESSAGE FUNCTIONS ======

// Helper function to send CAN message with error checking
void sendCANMessage(CAN_message_t &msg) {
  static uint32_t lastCANError = 0;
  
  if (!CAN.beginPacket(msg.id)) {
    if (millis() - lastCANError > 1000) { // Log error only once per second
      Serial.println("CAN beginPacket failed");
      lastCANError = millis();
    }
    return;
  }
  
  for (int i = 0; i < msg.len; i++) {
    CAN.write(msg.buf[i]);
  }
  
  if (!CAN.endPacket()) {
    if (millis() - lastCANError > 1000) { // Log error only once per second
      Serial.println("CAN endPacket failed");
      lastCANError = millis();
    }
  }
}

// Send CAN messages at 50Hz rate to BMW cluster
void SendData()
{
  DEBUG_PRINTLN("SEND CAN DATA");
  if (ascMSG) {
    CAN_msg_RPM.buf[0]= 0x05;
  }
  else {
    CAN_msg_RPM.buf[0]= 0x01;
  }	
  CAN_msg_RPM.buf[2]= rpmLSB; // RPM LSB
  CAN_msg_RPM.buf[3]= rpmMSB; // RPM MSB

  sendCANMessage(CAN_msg_RPM);

  // Send fuel consumption and error lights
  if (CEL < 200){  
    if (CEL < 100){
      CAN_msg_MPG_CEL.buf[0]= 0x12;  // keep CEL And EML on for about 2 seconds
    }
    else{
      CAN_msg_MPG_CEL.buf[0]= 0x02;  // keep CEL on for about 4 seconds
      }
    CEL++;
    }
  else{
    CAN_msg_MPG_CEL.buf[0]= 0x00;  // CEL off
  }

  if (currentStatus.spark == 5) {
  handleEMLLamp();
  }

  // TODO: Handle low battery warning
  // if (currentStatus.battery10 < 100) {
  //   // Handle low battery warning
  // }

  // This updates the fuel consumption counter. It's how much fuel is injected to engine, so PW and RPM affects it.
  updatePW = updatePW + ( currentStatus.PW1 * (currentStatus.RPM/10) );
  // We adjust the counter reading so that we get correct values sent to cluster. PW_ADJUST is just trial and error value. 
  PWcount = updatePW / PW_ADJUST;
  // Fuel consumption counter is 2-bytes so if the current value is higher than that, we roll over the counter.
  if (PWcount > 0xFFFF)
  {
    PWcount = PWcount - 0xFFFF;
    updatePW = updatePW - (0xFFFF * PW_ADJUST);
  }
  pwMSB = highByte(uint16_t(PWcount));  // split to high and low byte
  pwLSB = lowByte(uint16_t(PWcount));
  CAN_msg_MPG_CEL.buf[1]= pwLSB;  // LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2]= pwMSB;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3]= tempLight ;  // Overheat light
  sendCANMessage(CAN_msg_MPG_CEL);
  
  //Send CLT and TPS
  CAN_msg_CLT_TPS.buf[1] = CLT; // Coolant temp
  CAN_msg_CLT_TPS.buf[5] = TPS; // TPS value
  //Multiplexed Information in byte0
  switch (multiplex) {
    case 0: //CAN_LEVEL
      CAN_msg_CLT_TPS.buf[0] = 0x11;
      break;
    case 1: //OBD_STEUER
      if (currentStatus.RPM < 400)
      {
        CAN_msg_CLT_TPS.buf[0] = 0x80;
      }
      else
      {
        CAN_msg_CLT_TPS.buf[0] = 0x86;
      }
      break;
    case 2: //MD_NORM
      CAN_msg_CLT_TPS.buf[0] = 0xD9;
      break;
    default:
      CAN_msg_CLT_TPS.buf[0] = 0x11;
      break;
  }
  sendCANMessage(CAN_msg_CLT_TPS);

  MSGcounter++;
  if (MSGcounter >= 8)
  {
    multiplex++;
    if (multiplex >= 3)
    {
      multiplex = 0;
    }
    MSGcounter = 0;
  }
}
// ====== END CAN MESSAGE FUNCTIONS ======

// ====== WARNING LIGHT CONTROL FUNCTIONS ======

// Check 3 seconds CEL light timer and turn off the light if needed
void handleCelLamp() {
  static bool celActive = false;
  static uint32_t celStartTime = 0;

  if (!celActive) {
    // Activate CEL light
    celActive = true;
    celStartTime = millis();
    CAN_msg_MPG_CEL.buf[0] = 0x02; // Turn on CEL light
  } else {
    // Check if 3 seconds have passed
    if (millis() - celStartTime >= 3000) {
      CAN_msg_MPG_CEL.buf[0] = 0x00; // Turn off CEL light
      celActive = false;
    }
  }
}
// ===============================

// Check 3 seconds EML and CEL light timer and turn off the light if needed
void handleCelELMLamp() {
  static bool celActive = false;
  static uint32_t celStartTime = 0;

  if (!celActive) {
    // Activate both CEL and EML lights
    celActive = true;
    celStartTime = millis();
    CAN_msg_MPG_CEL.buf[0] = 0x12; // Turn on both lights
  } else {
    // Check if 3 seconds have passed
    if (millis() - celStartTime >= 3000) {
      CAN_msg_MPG_CEL.buf[0] = 0x00; // Turn off both lights
      celActive = false;
    }
  }
}
// ===============================

// Check 3 seconds EML light timer and turn off the light if needed
void handleEMLLamp() {
  static bool celActive = false;
  static uint32_t celStartTime = 0;

  if (!celActive) {
    // Activate EML light
    celActive = true;
    celStartTime = millis();
    CAN_msg_MPG_CEL.buf[0] = 0x10; // Turn on EML light
  } else {
    // Check if 3 seconds have passed
    if (millis() - celStartTime >= 3000) {
      CAN_msg_MPG_CEL.buf[0] = 0x00; // Turn off EML light
      celActive = false;
    }
  }
}
// ===============================

// Handle overheat light behavior based on coolant temperature
void handleHotBlink(uint8_t temp) {
  int16_t correctedTemp = temp - CLT_TEMP_OFFSET;
  static uint32_t lastBlink = 0;
  static bool lampOn = false;

  if (correctedTemp >= HOT_TEMP_SOLID) {
    tempLight = 0x8; // light on solid
    return;
  }
  
  if (BLINKING == 1 && correctedTemp >= HOT_TEMP_BLINK) {
    // If higher temp, the light blinks more frequently (e.g., every 1000ms at 100°C, every 200ms at 119°C)
    uint16_t interval = map(correctedTemp, HOT_TEMP_BLINK, HOT_TEMP_SOLID - 1, 1000, 200);
    if (millis() - lastBlink > interval) {
      lampOn = !lampOn;
      tempLight = lampOn ? 0x8 : 0x0;
      lastBlink = millis();
    }
  } else {
    tempLight = 0x0; // off
  }
}
// ====== END WARNING LIGHT FUNCTIONS ======

// ====== MAIN SETUP FUNCTION ======
void setup(){
  Serial1.begin(SERIAL_BAUDRATE, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.begin(SERIAL_DEBUG_BAUDRATE);
  
  Serial.println("=== ESP32 BMW CAN Bridge Starting ===");
  Serial.printf("Serial1 pins: RX=%d, TX=%d\n", SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.printf("MCP2515 pins: CS=%d, INT=%d\n", MCP2515_CS_PIN, MCP2515_INT_PIN);
  Serial.printf("SPI pins: MOSI=%d, MISO=%d, SCK=%d\n", MCP2515_MOSI_PIN, MCP2515_MISO_PIN, MCP2515_SCK_PIN);
  
  doRequest = false; // avoid sending A command before speeduino is ready
  rRequestCounter = 0;

  dash.begin(); // Initialize Bluetooth dashboard

  Serial.println("Initializing SPI and MCP2515 CAN controller...");
  
  // Initialize SPI with custom pins to avoid conflict with Serial1
  SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);
  
  // Set MCP2515 pins
  CAN.setPins(MCP2515_CS_PIN, MCP2515_INT_PIN);
  
  // Set clock frequency if your MCP2515 module uses a different crystal
  // Most modules use 8MHz, but some use 16MHz
  // CAN.setClockFrequency(8E6);  // Uncomment if using 8MHz crystal
  // CAN.setClockFrequency(16E6); // Uncomment if using 16MHz crystal
  
  // Initialize CAN bus at 500kbps
  if (!CAN.begin(CAN_BAUDRATE)) {
    Serial.println("Starting MCP2515 failed!");
    Serial.println("Check wiring and crystal frequency!");
    while (1);
  }
  Serial.println("MCP2515 CAN controller initialized successfully!");

  CAN_msg_RPM.len = 8;
  CAN_msg_CLT_TPS.len = 8;
  CAN_msg_MPG_CEL.len = 8;
  CAN_msg_RPM.id = 0x316; // CAN ID for RPM message
  CAN_msg_CLT_TPS.id = 0x329; // CAN ID for CLT and TPS message
  CAN_msg_MPG_CEL.id = 0x545; // CAN ID for fuel consumption and CEL light

  // Send this message to get rid of EML light and also set the static values for the message
  CAN_msg_MPG_CEL.buf[0] = 0x02;  // error State
  CAN_msg_MPG_CEL.buf[1] = 0x00;  // LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2] = 0x00;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3] = 0x00;  // Overheat light
  CAN_msg_MPG_CEL.buf[4] = 0x00;  // not used, but set to zero just in case
  CAN_msg_MPG_CEL.buf[5] = 0x00;  // not used, but set to zero just in case
  CAN_msg_MPG_CEL.buf[6] = 0x00;  // not used, but set to zero just in case
  CAN_msg_MPG_CEL.buf[7] = 0x00;  // not used, but set to zero just in case

  // Set the static values for the other two messages
  CAN_msg_RPM.buf[0] = 0x01;  // bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
  CAN_msg_RPM.buf[1] = 0x0C;  // Indexed Engine Torque in % of C_TQ_STND TODO: do torque calculation
  CAN_msg_RPM.buf[4] = 0x0C;  // Indicated Engine Torque in % of C_TQ_STND TODO: do torque calculation - Use same as for byte 1
  CAN_msg_RPM.buf[5] = 0x15;  // Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
  CAN_msg_RPM.buf[6] = 0x00;  // not used
  CAN_msg_RPM.buf[7] = 0x35;  // Theoretical Engine Torque in % of C_TQ_STND after charge intervention

  CAN_msg_CLT_TPS.buf[0] = 0x11;  // Multiplexed Information
  CAN_msg_CLT_TPS.buf[2] = 0xB2;  // CLT temp
  CAN_msg_CLT_TPS.buf[3] = 0x00;  // Baro
  CAN_msg_CLT_TPS.buf[4] = 0x08;  // bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_CLT_TPS.buf[6] = 0x00;  // TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_CLT_TPS.buf[7] = 0x00;  // not used, but set to zero just in case

  // Start with sensible values for some of these variables.
  CLT = 60;
  currentStatus.PW1 = 0;
  updatePW = 0;
  rpmLSB = 0;
  rpmMSB = 0;
  pwLSB = 0;
  pwMSB = 0;
  CEL = 0;
  tempLight = 0;
  SerialState = NOTHING_RECEIVED;
  data_error = false;
  responseSent = false;
  newData = false;
  MSGcounter = 0;
  multiplex = 0;
  ascMSG = false;
  radOutletTemp = 0;
  oilTemp = 0;
  PWMfanDuty = 10; // minimum valid duty is 10%
  acBitfield = 0;
  acBitfield2 = 0;
  eFanBitfield = 0;

  // Configure Task Watchdog Timer - enable basic watchdog
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 10000, // 10 second timeout (increased from 30s)
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Bitmask of all cores
    .trigger_panic = false // Disable panic to avoid hard resets
  };
  esp_task_wdt_init(&twdt_config);
  
  Serial.println("Task Watchdog configured");
  Serial.println("Version date: 18.09.2025 - Fixed blocking issues"); 
  doRequest = true; 
  delay(2000);
  rRequestCounter = SerialUpdateRate;
  xTaskCreatePinnedToCore(mainTask, "MainTask", 40960, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(canTask, "CANTask", 10240, NULL, 1, NULL, 1);
  
  // Add tasks to watchdog (optional, only if you want to monitor them)
  // esp_task_wdt_add(NULL); // Add current task to WDT
  
  Serial.print("Tasks created - Free heap: "); Serial.println(ESP.getFreeHeap());
  Serial.println("=== System fully initialized ===");
}
// ====== END SETUP FUNCTION ======

// ====== CAN MESSAGE READING FUNCTIONS ======
void readCanMessage() {
  DEBUG_PRINTLN("READ CAN DATA");
  
  // Add timeout to prevent infinite blocking
  uint32_t startTime = millis();
  const uint32_t CAN_READ_TIMEOUT = 10; // 10ms timeout
  
  while (CAN.parsePacket() && (millis() - startTime < CAN_READ_TIMEOUT)) {
    CAN_inMsg.id = CAN.packetId();
    CAN_inMsg.len = CAN.packetDlc();
    
    // Validate message length
    if (CAN_inMsg.len > 8) {
      Serial.println("Invalid CAN message length");
      continue;
    }
    
    for (int i = 0; i < CAN_inMsg.len; i++) {
      CAN_inMsg.buf[i] = CAN.read();
    }
    
    switch (CAN_inMsg.id)
    {
      case 0x613:
        odometerLSB = CAN_inMsg.buf[0];
        odometerMSB = CAN_inMsg.buf[1];
        FuelLevel = CAN_inMsg.buf[2];
        runningClock = ((CAN_inMsg.buf[4] << 8) | (CAN_inMsg.buf[3]));
      break;
      case 0x615:
        ambientTemp = CAN_inMsg.buf[3];
        acBitfield = CAN_inMsg.buf[0];
        acBitfield2 = CAN_inMsg.buf[4];
        eFanBitfield = CAN_inMsg.buf[1];
      break;
      case  0x153:
        ascMSG = true;
        VSS = ((CAN_inMsg.buf[2] << 8) | (CAN_inMsg.buf[1]));
        VSS = VSS >> 7;
        VSS = VSS - 2;
      break;
      case  0x1F0:
        VSS1 = ((CAN_inMsg.buf[1] << 8) | (CAN_inMsg.buf[0]));
        VSS1 = VSS1 >> 4;
        VSS1 = VSS1 - 2;
        VSS2 = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[2]));
        VSS2 = VSS2 >> 4;
        VSS2 = VSS2 - 2;
        VSS3 = ((CAN_inMsg.buf[5] << 8) | (CAN_inMsg.buf[4]));
        VSS3 = VSS3 >> 4;
        VSS3 = VSS3 - 2;
        VSS4 = ((CAN_inMsg.buf[7] << 8) | (CAN_inMsg.buf[6]));
        VSS4 = VSS4 >> 4;
        VSS4 = VSS4 - 2;
      break;
      default:
      break;
    }
  }
}

// ====== SPEEDUINO DATA EXCHANGE FUNCTIONS ======

// This function sends the requested data back to speeduino when it requests it
void SendDataToSpeeduino(){
  // Add timeout protection
  static uint32_t lastSendTime = 0;
  if (millis() - lastSendTime < 10) { // Minimum 10ms between sends
    return;
  }
  lastSendTime = millis();
  
  if (!Serial1) {
    Serial.println("Serial1 not available");
    return;
  }
  
  Serial1.write("G");                      // reply "G" cmd
  switch (CanAddress)
  {
    case 0x613:  // Odometer and fuel level
      Serial1.write(1);                        // send 1 to confirm cmd received and valid
      Serial1.write(canin_channel);            // confirms the destination channel
      Serial1.write(odometerLSB);              // write back the requested data
      Serial1.write(odometerMSB);
      Serial1.write(FuelLevel);
      Serial1.write(lowByte(runningClock));
      Serial1.write(highByte(runningClock));
      for (int i=0; i<3; i++) { Serial1.write(0); }                // Rest will be zero
    break;
    case 0x615:  // Ambient temp
      Serial1.write(1);                        // send 1 to confirm cmd received and valid
      Serial1.write(canin_channel);            // confirms the destination channel
      for (int i=0; i<3; i++) { Serial1.write(0); }
      Serial1.write(ambientTemp);              // write back the requested data
        for (int i=0; i<4; i++) { Serial1.write(0); }
    break;
    case  0x153:  // VSS
      Serial1.write(1);                        // send 1 to confirm cmd received and valid
      Serial1.write(canin_channel);            // confirms the destination channel
      Serial1.write(0);
      Serial1.write(lowByte(VSS));
      Serial1.write(highByte(VSS));
      for (int i=0; i<5; i++) { Serial1.write(0); }
    break;
    case  0x1F0:  // VSS for each invidual wheel
      Serial1.write(1);                        // send 1 to confirm cmd received and valid
      Serial1.write(canin_channel);            // confirms the destination channel               //write back the requested data
      Serial1.write(lowByte(VSS1));
      Serial1.write(highByte(VSS1));
      Serial1.write(lowByte(VSS2));
      Serial1.write(highByte(VSS2));
      Serial1.write(lowByte(VSS3));
      Serial1.write(highByte(VSS3));
      Serial1.write(lowByte(VSS4));
      Serial1.write(highByte(VSS4));
    break;
    default:
      Serial1.write(0);                        // send 0 to confirm cmd received but not valid
      Serial1.write(canin_channel);            // destination channel
      for (int i=0; i<8; i++) { Serial1.write(0); }  // we need to still write some data as a response, or real time data reading will slow down significantly
    break;
  }
  Serial1.flush(); // Ensure data is sent
}

// ====== DATA PROCESSING FUNCTIONS ======

// Display the needed values in serial monitor for debugging
void displayData(){
  DEBUG_PRINT("RPM-"); DEBUG_PRINT(currentStatus.RPM); DEBUG_PRINT("\t");
  DEBUG_PRINT("PW-"); DEBUG_PRINT(currentStatus.PW1); DEBUG_PRINT("\t");
  DEBUG_PRINT("CLT-"); DEBUG_PRINT(currentStatus.CLT - 40); DEBUG_PRINT("\t");
  DEBUG_PRINT("TPS-"); DEBUG_PRINT(currentStatus.TPS); DEBUG_PRINT("\t");
}

// ====== SPEEDUINO MESSAGE HANDLERS ======

// This function processes the data received from speeduino and converts it to usable format
void processData(){
  DEBUG_PRINTLN("PROCESS DATA FROM SPEEDUINO");
  unsigned int tempRPM;
  data_error = false; // set the received data as ok
  static uint32_t corruptedDataCount = 0;

  // Basic sanity check on first few bytes to detect corruption early
  if (SpeedyResponse[0] > 240 || SpeedyResponse[1] > 240 || SpeedyResponse[2] > 240) {
    Serial.printf("Data corruption detected in header bytes: %d, %d, %d\n", 
                  SpeedyResponse[0], SpeedyResponse[1], SpeedyResponse[2]);
    data_error = true;
    corruptedDataCount++;
    return;
  }

  currentStatus.secl = SpeedyResponse[0];
  currentStatus.status1 = SpeedyResponse[1];
  currentStatus.engine = SpeedyResponse[2];
  currentStatus.dwell = SpeedyResponse[3];
  currentStatus.MAP = ((SpeedyResponse [5] << 8) | (SpeedyResponse [4]));
  currentStatus.IAT = SpeedyResponse[6];
  currentStatus.CLT = SpeedyResponse[7];
  currentStatus.batCorrection = SpeedyResponse[8];
  currentStatus.battery10 = SpeedyResponse[9];
  currentStatus.O2 = SpeedyResponse[10];
  currentStatus.egoCorrection = SpeedyResponse[11];
  currentStatus.iatCorrection = SpeedyResponse[12];
  currentStatus.wueCorrection = SpeedyResponse[13];
  currentStatus.RPM = ((SpeedyResponse [15] << 8) | (SpeedyResponse [14])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.AEamount = SpeedyResponse[16];
  currentStatus.corrections = SpeedyResponse[17];
  currentStatus.VE = SpeedyResponse[18];
  currentStatus.afrTarget = SpeedyResponse[19];
  currentStatus.PW1 = ((SpeedyResponse [21] << 8) | (SpeedyResponse [20])); // PW low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.tpsDOT = SpeedyResponse[22];
  currentStatus.advance = SpeedyResponse[23];
  currentStatus.TPS = SpeedyResponse[24];
  currentStatus.loopsPerSecond = ((SpeedyResponse [26] << 8) | (SpeedyResponse [25]));
  currentStatus.freeRAM = ((SpeedyResponse [28] << 8) | (SpeedyResponse [27]));
  currentStatus.boostTarget = SpeedyResponse[29]; // boost target divided by 2 to fit in a byte
  currentStatus.boostDuty = SpeedyResponse[30];
  currentStatus.spark = SpeedyResponse[31]; // Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
  currentStatus.rpmDOT = ((SpeedyResponse [33] << 8) | (SpeedyResponse [32]));
  currentStatus.ethanolPct = SpeedyResponse[34]; // Flex sensor value (or 0 if not used)
  currentStatus.flexCorrection = SpeedyResponse[35]; // Flex fuel correction (% above or below 100)
  currentStatus.flexIgnCorrection = SpeedyResponse[36]; // Ignition correction (Increased degrees of advance) for flex fuel
  currentStatus.idleLoad = SpeedyResponse[37];
  currentStatus.testOutputs = SpeedyResponse[38]; // testEnabled(0), testActive(1)
  currentStatus.O2_2 = SpeedyResponse[39]; // O2
  currentStatus.baro = SpeedyResponse[40]; // Barometer value
  currentStatus.CANin_1 = ((SpeedyResponse [42] << 8) | (SpeedyResponse [41]));
  currentStatus.CANin_2 = ((SpeedyResponse [44] << 8) | (SpeedyResponse [43]));
  currentStatus.CANin_3 = ((SpeedyResponse [46] << 8) | (SpeedyResponse [45]));
  currentStatus.CANin_4 = ((SpeedyResponse [48] << 8) | (SpeedyResponse [47]));
  currentStatus.CANin_5 = ((SpeedyResponse [50] << 8) | (SpeedyResponse [49]));
  currentStatus.CANin_6 = ((SpeedyResponse [52] << 8) | (SpeedyResponse [51]));
  currentStatus.CANin_7 = ((SpeedyResponse [54] << 8) | (SpeedyResponse [53]));
  currentStatus.CANin_8 = ((SpeedyResponse [56] << 8) | (SpeedyResponse [55]));
  currentStatus.CANin_9 = ((SpeedyResponse [58] << 8) | (SpeedyResponse [57]));
  currentStatus.CANin_10 = ((SpeedyResponse [60] << 8) | (SpeedyResponse [59]));
  currentStatus.CANin_11 = ((SpeedyResponse [62] << 8) | (SpeedyResponse [61]));
  currentStatus.CANin_12 = ((SpeedyResponse [64] << 8) | (SpeedyResponse [63]));
  currentStatus.CANin_13 = ((SpeedyResponse [66] << 8) | (SpeedyResponse [65]));
  currentStatus.CANin_14 = ((SpeedyResponse [68] << 8) | (SpeedyResponse [67]));
  currentStatus.CANin_15 = ((SpeedyResponse [70] << 8) | (SpeedyResponse [69]));
  currentStatus.CANin_16 = ((SpeedyResponse [71] << 8) | (SpeedyResponse [71]));
  currentStatus.tpsADC = SpeedyResponse[73];

  // Additional sanity checks on key values - adjusted for realistic Speeduino values
  // MAP: 0-500 kPa is normal, IAT: -40 to +100°C, Battery: 0-25.5V (battery10 is voltage*10)
  if (currentStatus.MAP > 600 || currentStatus.IAT > 200 || currentStatus.battery10 > 255) {
    Serial.printf("Suspicious data values: MAP=%d, IAT=%d, Bat=%d\n", 
                  currentStatus.MAP, currentStatus.IAT, currentStatus.battery10);
    data_error = true;
    corruptedDataCount++;
    return;
  }

  // Report corruption statistics periodically
  if (corruptedDataCount > 0 && corruptedDataCount % 10 == 0) {
    Serial.printf("Total corrupted data packets: %lu\n", corruptedDataCount);
  }

  // Check if received values make sense and convert those if all is ok.
  if (currentStatus.RPM < MAX_ENGINE_RPM && data_error == false)  // the engine will not probably rev over 8000 RPM
  {
    tempRPM = currentStatus.RPM * RPM_CONVERSION_FACTOR; // RPM conversion factor for e46/e39 cluster
    rpmMSB = tempRPM >> 8;  // split to high and low byte
    rpmLSB = tempRPM;
  }
  else
  {
    data_error = true; // data received is probably corrupted, don't use it.
  }
  if (currentStatus.CLT < MAX_CLT_TEMP && data_error == false)  // 142 degrees Celsius is the hottest temp that fits to the conversion. 
  {
    CLT = (currentStatus.CLT - CLT_TEMP_OFFSET) * 4 / 3 + 64;  // CLT conversion factor for e46/e39 cluster
    handleHotBlink(currentStatus.CLT);
  }
  else
  {
    data_error = true;  // data received is probably corrupted, don't use it.
  }

  if (currentStatus.TPS < (MAX_TPS_VALUE + 1) && data_error == false)  // TPS values can only be from 0-200 (previously this was from 0-100 on speeduino)
  {
    TPS = map(currentStatus.TPS, 0, MAX_TPS_VALUE, 0, 254); // 0-200 TPS value mapped to 0x00 to 0xFE range.
    newData = true; // we have now new data and it passes the checks.
  }
  else
  {
    data_error = true; // data received is probably corrupted, don't use it.
  }
}

// ====== SERIAL MESSAGE HANDLING ======

// Handle A-message from speeduino. This contains real time data.
void HandleA()
{ 
  DEBUG_PRINTLN("HANDLE A");
  
  // Add timeout protection for reading serial data
  uint32_t startTime = millis();
  const uint32_t SERIAL_READ_TIMEOUT = 100; // 100ms timeout
  static uint32_t successfulReads = 0;
  
  Serial1.print("A");
  data_error = false;
  
  for (int i = 0; i < 75; i++) {
    uint32_t waitStart = millis();
    while (!Serial1.available()) {
      if (millis() - startTime > SERIAL_READ_TIMEOUT) {
        Serial.printf("Serial read timeout in HandleA at byte %d (waited %lums)\n", i, millis() - waitStart);
        data_error = true;
        SerialState = NOTHING_RECEIVED;
        
        // Flush remaining data to prevent desync
        while (Serial1.available()) Serial1.read();
        return;
      }
      yield(); // Allow other tasks to run
    }
    SpeedyResponse[i] = Serial1.read();
  }
  
  successfulReads++;
  if (successfulReads % 1000 == 0) { // Log every 1000 successful reads (reduced from 100)
    Serial.printf("HandleA: %lu successful reads completed\n", successfulReads);
  }
  
  processData();                  // do the necessary processing for received data
  doRequest = true;               // restart data reading
  oldtime = millis();             // zero the timeout
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}
//================================

// Handle R-message from speeduino. This is a request for some data.
void HandleR()
{
  Serial1.println("R");
  
  // Add timeout protection for reading serial data
  uint32_t startTime = millis();
  const uint32_t SERIAL_READ_TIMEOUT = 100; // 100ms timeout
  
  byte tmp0;
  byte tmp1;
  
  // Read with timeout protection
  while (!Serial1.available()) {
    if (millis() - startTime > SERIAL_READ_TIMEOUT) {
      Serial.println("Serial read timeout in HandleR - channel");
      SerialState = NOTHING_RECEIVED;
      return;
    }
    yield();
  }
  canin_channel = Serial1.read();
  
  while (!Serial1.available()) {
    if (millis() - startTime > SERIAL_READ_TIMEOUT) {
      Serial.println("Serial read timeout in HandleR - tmp0");
      SerialState = NOTHING_RECEIVED;
      return;
    }
    yield();
  }
  tmp0 = Serial1.read();  // read in lsb of source can address
  
  while (!Serial1.available()) {
    if (millis() - startTime > SERIAL_READ_TIMEOUT) {
      Serial.println("Serial read timeout in HandleR - tmp1");
      SerialState = NOTHING_RECEIVED;
      return;
    }
    yield();
  }
  tmp1 = Serial1.read();  // read in msb of source can address
  
  CanAddress = tmp1<<8 | tmp0;
  SendDataToSpeeduino();  // send the data to speeduino
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

// Read the first byte from serial to determine what kind of message it is
void ReadSerial()
{
  DEBUG_PRINTLN("READ SERIAL");
  static uint32_t invalidMessageCount = 0;
  static uint32_t lastFlushTime = 0;
  
  currentCommand = Serial1.read();
  switch (currentCommand)
  {
    case 'A':  // Speeduino sends data in A-message
      SerialState = A_MESSAGE;
      invalidMessageCount = 0; // Reset counter on valid message
    break;
    case 'R':  // Speeduino requests data in R-message
      SerialState = R_MESSAGE;
      invalidMessageCount = 0; // Reset counter on valid message
    break;
    default:
      invalidMessageCount++;
      Serial.printf("Invalid message #%lu: C %d (0x%02X)\n", invalidMessageCount, currentCommand, currentCommand);
      
      // If we get too many invalid messages in a row, flush the buffer to resync
      if (invalidMessageCount >= 5 || (millis() - lastFlushTime > 1000)) {
        Serial.println("Flushing Serial1 buffer to resync communication");
        
        // Flush input buffer
        while (Serial1.available()) {
          Serial1.read();
          delay(1); // Small delay to ensure we read everything
        }
        
        SerialState = NOTHING_RECEIVED;
        invalidMessageCount = 0;
        lastFlushTime = millis();
        
        // Force a new request after buffer flush
        doRequest = true;
        oldtime = millis();
      }
    break;
  }
}

// ====== MAIN LOOP ======

// Main loop - suspended as all work is done in FreeRTOS tasks
void loop() {
  vTaskDelay(portMAX_DELAY); // Suspend main loop indefinitely
}
// ====== END MAIN LOOP ======
