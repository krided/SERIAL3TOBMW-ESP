/*
  BMW E39/E46 CAN Bridge & Dashboard for ESP32 WROOM32D + MCP2551
  ----------------------------------------------------------------

  This project is an ESP32-based solution that reads real-time data from
  a Speeduino EFI system via Serial3 and converts it into CAN messages 
  compatible with BMW E39/E46 instrument clusters. It also provides a
  customizable dashboard, supporting Bluetooth telemetry for mobile devices.

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
[CAN@0.3.1] - Arduino CAN library for ESP32
[Ticker@3.3.0] - ESP32 Ticker library
[BluetoothSerial@3.3.0] - Bluetooth Serial library for ESP32
*/

// ========== Library ===========
#include "config.h" // configuration file
#include <Arduino.h>
#include <CAN.h>
#include <Ticker.h>
#include "dashboard.h" // for wifi dashboard
// ===============================

// ====== GLOBAL VARIABLES FOR DASHBOARD ======
Dashboard dash;
// ==========================================

// ======DONT WORRY ABOUT THAT ====
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : BIT_CLEAR((var), (pos)))

#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define A_MESSAGE               2
#define PWM_MESSAGE             3
// ==============================

// Struct for CAN messages
struct CAN_message_t {
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
};
static CAN_message_t CAN_msg_RPM;
static CAN_message_t CAN_msg_CLT_TPS;
static CAN_message_t CAN_msg_MPG_CEL;
static CAN_message_t CAN_inMsg;
// ===============================

// ====== GLOBAL VARIABLES ======
unsigned long lastDashUpdate = 0; // last time dashboard was updated
const unsigned long dashupdaterate = DASH_UPLOAD_MS - DASH_UPDATE_MS; // updating data in dashboard with rate defined in config.h

// This struct gathers data read from speeduino. This is really just direct copy of what speeduino has internally
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

static uint32_t oldtime=millis();   // for the timeout
uint8_t SpeedyResponse[100]; //The data buffer for the serial3 data. This is longer than needed, just in case
uint8_t rpmLSB;   // Least significant byte for RPM message
uint8_t rpmMSB;  // Most significant byte for RPM message
uint8_t pwLSB;   // Least significant byte for PW message
uint8_t pwMSB;  // Most significant byte for PW message
uint8_t CEL;   //timer for how long CEL light be kept on
uint32_t updatePW;
uint8_t odometerLSB;
uint8_t odometerMSB;
uint8_t FuelLevel;
uint8_t ambientTemp;
int CLT; // to store coolant temp
uint32_t PWcount;
uint8_t TPS,tempLight; // TPS value and overheat light on/off
bool data_error; //indicator for the data from speeduino being ok.
bool responseSent; // to keep track if we have responded to data request or not.
bool newData; // This tells if we have new data available from speeduino or not.
bool ascMSG; // ASC message received.
uint8_t rRequestCounter; // only request PWM fan duty from speeduino once in a second
uint8_t PWMfanDuty; // PWM fan duty
uint8_t data[255]; // For DS2 data
uint8_t SerialState,canin_channel,currentCommand;
uint16_t CanAddress,runningClock;
uint16_t VSS,VSS1,VSS2,VSS3,VSS4;
uint8_t MSGcounter; //this keeps track of which multiplexed info is sent in 0x329 byte 0
uint8_t multiplex;
uint8_t radOutletTemp;
uint8_t oilTemp;
uint8_t acBitfield;
uint8_t acBitfield2;
uint8_t eFanBitfield;
bool doRequest = true;

Ticker requestTicker;
Ticker sendTicker;
// ===============================

// Request data from speeduino at defined rate
void requestData() {
  if (doRequest){
    Serial2.write("A"); // Send A to request real time data
    doRequest = false;
  }
}
// ===============================

// Send the data to speeduino when it requests it
void SendData()   // Send can messages in 50Hz phase from timer interrupt. This is important to be high enough Hz rate to make cluster work smoothly.
{
  if (ascMSG) {
    CAN_msg_RPM.buf[0]= 0x05;
  }
  else {
    CAN_msg_RPM.buf[0]= 0x01;
  }	
  CAN_msg_RPM.buf[2]= rpmLSB; // RPM LSB
  CAN_msg_RPM.buf[3]= rpmMSB; // RPM MSB

  CAN.beginPacket(CAN_msg_RPM.id);
  for (int i=0; i<CAN_msg_RPM.len; i++) CAN.write(CAN_msg_RPM.buf[i]);
  CAN.endPacket();

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

  //To do: Handle low battery warning
  //if (currentStatus.battery10 < 100) {
  //  // Handle low battery warning
  //}

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
  CAN.beginPacket(CAN_msg_MPG_CEL.id);
  for (int i=0; i<CAN_msg_MPG_CEL.len; i++) CAN.write(CAN_msg_MPG_CEL.buf[i]);
  CAN.endPacket();
  
  //Send CLT and TPS
  
  CAN_msg_CLT_TPS.buf[1]= CLT; // Coolant temp
  CAN_msg_CLT_TPS.buf[5]= TPS; // TPS value.
    //Multiplexed Information in byte0
  switch (multiplex) {
  case 0: //CAN_LEVEL
    CAN_msg_CLT_TPS.buf[0]= 0x11;
    break;
  case 1: //OBD_STEUER
    if (currentStatus.RPM < 400)
    {
      CAN_msg_CLT_TPS.buf[0]= 0x80;
    }
    else
    {
      CAN_msg_CLT_TPS.buf[0]= 0x86;
    }
    break;
  case 2: //MD_NORM
    CAN_msg_CLT_TPS.buf[0]= 0xD9;
    break;
  default:
    CAN_msg_CLT_TPS.buf[0]= 0x11;
    break;
  }
  CAN.beginPacket(CAN_msg_CLT_TPS.id);
  for (int i=0; i<CAN_msg_CLT_TPS.len; i++) CAN.write(CAN_msg_CLT_TPS.buf[i]);
  CAN.endPacket();

  MSGcounter++;
  if (MSGcounter >= 8)
  {
    multiplex++;
      if (multiplex >= 3)
      {
        multiplex = 0;
      }
    {
      MSGcounter = 0;
    }
  }
}
// ===============================  

// Check  3 seconds CEL light timer and turn off the light if needed
void handleCelLamp() {
  static bool celActive = false;
  static uint32_t celStartTime = 0;

  if (!celActive) {
    // Wywołaj, aby zapalić lampkę
    celActive = true;
    celStartTime = millis();
    CAN_msg_MPG_CEL.buf[0] = 0x02; // zapal lampkę
  } else {
    // Sprawdzaj w pętli, czy minęły 3 sekundy
    if (millis() - celStartTime >= 3000) {
      CAN_msg_MPG_CEL.buf[0] = 0x00; // zgaś lampkę
      celActive = false;
    }
  }
}
// ===============================

// Check  3 seconds EML and CEL light timer and turn off the light if needed
void handleCelELMLamp() {
  static bool celActive = false;
  static uint32_t celStartTime = 0;

  if (!celActive) {
    // Wywołaj, aby zapalić lampkę
    celActive = true;
    celStartTime = millis();
    CAN_msg_MPG_CEL.buf[0] = 0x12; // zapal lampkę
  } else {
    // Sprawdzaj w pętli, czy minęły 3 sekundy
    if (millis() - celStartTime >= 3000) {
      CAN_msg_MPG_CEL.buf[0] = 0x00; // zgaś lampkę
      celActive = false;
    }
  }
}
// ===============================

// Check  3 seconds EML light timer and turn off the light if needed
void handleEMLLamp() {
  static bool celActive = false;
  static uint32_t celStartTime = 0;

  if (!celActive) {
    // Wywołaj, aby zapalić lampkę
    celActive = true;
    celStartTime = millis();
    CAN_msg_MPG_CEL.buf[0] = 0x10; // zapal lampkę
  } else {
    // Sprawdzaj w pętli, czy minęły 3 sekundy
    if (millis() - celStartTime >= 3000) {
      CAN_msg_MPG_CEL.buf[0] = 0x00; // zgaś lampkę
      celActive = false;
    }
  }
}
// ===============================

// Handle overheat light behavior based on coolant temperature
void handleHotBlink(uint8_t temp) {
  static uint32_t lastBlink = 0;
  static bool lampOn = false;

    if (temp >= HOT_TEMP_SOLID) {
      tempLight = 0x8; // light on solid
      return;
    }
    if (BLINKING == 1)
    {
      if (temp >= HOT_TEMP_BLINK) {
      // If higher temp, the light blinks more frequently (e.g., every 1000ms at 100°C, every 200ms at 119°C)
      uint16_t interval = map(temp, HOT_TEMP_BLINK, HOT_TEMP_SOLID-1, 1000, 200);
      if (millis() - lastBlink > interval) {
        lampOn = !lampOn;
        tempLight = lampOn ? 0x8 : 0x0;
        lastBlink = millis();
      }
    }
    else {
      tempLight = 0x0; // off
    }
  }
}
// ===============================

// Main setup function
void setup(){
  Serial2.begin(SERIAL_BAUDRATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN); // RX2=16, TX2=17 (change pins if needed)
  Serial.begin(SERIAL_DEBUG_BAUDRATE); // for debugging
  
  doRequest = false; // to avoid sending A command before speeduino is ready
  rRequestCounter = 0; // to reset the counter

  dash.begin(); // Initialize Bluetooth dashboard

  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN); // Set CAN pins
  if (!CAN.begin(CAN_BAUDRATE)) { // Start CAN bus at 500 kbps
    Serial.println("Starting CAN failed!");
    while (1);
  }

  CAN_msg_RPM.len = 8; // 8 bytes in can message
  CAN_msg_CLT_TPS.len = 8;
  CAN_msg_MPG_CEL.len = 8;
  CAN_msg_RPM.id = 0x316; // CAN ID for RPM message is 0x316
  CAN_msg_CLT_TPS.id = 0x329; // CAN ID for CLT and TSP message is 0x329
  CAN_msg_MPG_CEL.id = 0x545; // CAN ID for fuel consumption and CEl light is 0x545

  // send this message to get rid of EML light and also set the static values for the message
  CAN_msg_MPG_CEL.buf[0]= 0x02;  // error State
  CAN_msg_MPG_CEL.buf[1]= 0x00;  // LSB Fuel consumption
  CAN_msg_MPG_CEL.buf[2]= 0x00;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL.buf[3]= 0x00;  // Overheat light
  CAN_msg_MPG_CEL.buf[4]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[5]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[6]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL.buf[7]= 0x00; // not used, but set to zero just in case.

  // set the static values for the other two messages
  CAN_msg_RPM.buf[0]= 0x01;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
  CAN_msg_RPM.buf[1]= 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation!!
  CAN_msg_RPM.buf[4]= 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
  CAN_msg_RPM.buf[5]= 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
  CAN_msg_RPM.buf[6]= 0x00;  //not used
  CAN_msg_RPM.buf[7]= 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention

  CAN_msg_CLT_TPS.buf[0]= 0x11;  //Multiplexed Information
  CAN_msg_CLT_TPS.buf[2]= 0xB2;  //CLT temp
  CAN_msg_CLT_TPS.buf[3]= 0x00;  //Baro
  CAN_msg_CLT_TPS.buf[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_CLT_TPS.buf[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_CLT_TPS.buf[7]= 0x00;  //not used, but set to zero just in case.

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

  requestTicker.attach(1.0/SerialUpdateRate, requestData);
  sendTicker.attach(1.0/ClusterUpdateRate, SendData);

  Serial.println ("Version date: 30.08.2025"); 
  doRequest = true; 

  rRequestCounter = SerialUpdateRate;
}
// ===============================

// Main can reading function
void readCanMessage() {
  if (CAN.parsePacket()) {
    CAN_inMsg.id = CAN.packetId();
    CAN_inMsg.len = CAN.packetDlc();
    for (int i=0; i<CAN_inMsg.len; i++) {
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
//================================

// This function sends the requested data back to speeduino when it requests it
void SendDataToSpeeduino(){
  Serial2.write("G");                      // reply "G" cmd
  switch (CanAddress)
  {
    case 0x613:  // Odometer and fuel level
      Serial2.write(1);                        // send 1 to confirm cmd received and valid
      Serial2.write(canin_channel);            // confirms the destination channel
      Serial2.write(odometerLSB);              // write back the requested data
      Serial2.write(odometerMSB);
      Serial2.write(FuelLevel);
      Serial2.write(lowByte(runningClock));
      Serial2.write(highByte(runningClock));
      for (int i=0; i<3; i++) { Serial2.write(0); }                // Rest will be zero
    break;
    case 0x615:  // Ambient temp
      Serial2.write(1);                        // send 1 to confirm cmd received and valid
      Serial2.write(canin_channel);            // confirms the destination channel
      for (int i=0; i<3; i++) { Serial2.write(0); }
      Serial2.write(ambientTemp);              // write back the requested data
        for (int i=0; i<4; i++) { Serial2.write(0); }
    break;
    case  0x153:  // VSS
      Serial2.write(1);                        // send 1 to confirm cmd received and valid
      Serial2.write(canin_channel);            // confirms the destination channel
      Serial2.write(0);
      Serial2.write(lowByte(VSS));
      Serial2.write(highByte(VSS));
      for (int i=0; i<5; i++) { Serial2.write(0); }
    break;
    case  0x1F0:  // VSS for each invidual wheel
      Serial2.write(1);                        // send 1 to confirm cmd received and valid
      Serial2.write(canin_channel);            // confirms the destination channel               //write back the requested data
      Serial2.write(lowByte(VSS1));
      Serial2.write(highByte(VSS1));
      Serial2.write(lowByte(VSS2));
      Serial2.write(highByte(VSS2));
      Serial2.write(lowByte(VSS3));
      Serial2.write(highByte(VSS3));
      Serial2.write(lowByte(VSS4));
      Serial2.write(highByte(VSS4));
    break;
    default:
      Serial2.write(0);                        // send 0 to confirm cmd received but not valid
      Serial2.write(canin_channel);            // destination channel
      for (int i=0; i<8; i++) { Serial2.write(0); }                // we need to still write some crap as an response, or real time data reading will slow down significantly
      Serial.print ("Wrong CAN address");
    break;
  }
}
//================================

// display the needed values in serial monitor for debugging
void displayData(){
  Serial.print ("RPM-"); Serial.print (currentStatus.RPM); Serial.print("\t");
  Serial.print ("PW-"); Serial.print (currentStatus.PW1); Serial.print("\t");
  Serial.print ("PWcount-"); Serial.print (PWcount); Serial.print("\t");
  Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
  Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");

}
//================================

// This function processes the data received from speeduino and converts it to usable format
void processData(){   // necessary conversion for the data before sending to CAN BUS
  unsigned int tempRPM;
  data_error = false; // set the received data as ok

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

  // check if received values makes sense and convert those if all is ok.
  if (currentStatus.RPM < 8000 && data_error == false)  // the engine will not probaply rev over 8000 RPM
  {
    tempRPM = currentStatus.RPM * 6.4; // RPM conversion factor for e46/e39 cluster
    rpmMSB = tempRPM >> 8;  // split to high and low byte
    rpmLSB = tempRPM;
  }
  else
  {
    data_error = true; // data received is probaply corrupted, don't use it.
    Serial.print ("Error. RPM Received:"); Serial.print (currentStatus.RPM); Serial.print("\t");
  }

  if (currentStatus.CLT < 182 && data_error == false)  // 142 degrees Celcius is the hottest temp that fits to the conversion. 
  {
    CLT = (currentStatus.CLT -40)*4/3+64;  // CLT conversion factor for e46/e39 cluster
    handleHotBlink(currentStatus.CLT);
  }
  else
  {
    data_error = true;  // data received is probaply corrupted, don't use it.
    Serial.print ("Error. CLT received:"); Serial.print (currentStatus.CLT); Serial.print("\t");
  }

  if (currentStatus.TPS < 201 && data_error == false)  // TPS values can only be from 0-200 (previously this was from 0-100 on speeduino)
  {
    TPS = map(currentStatus.TPS, 0, 200, 0, 254); // 0-100 TPS value mapped to 0x00 to 0xFE range.
    newData = true; // we have now new data and it passes the checks.
  }
  else
  {
    data_error = true; // data received is probaply corrupted, don't use it.
    Serial.print ("Error. TPS received:"); Serial.print (currentStatus.TPS); Serial.print("\t");
  }
}
//================================

// Handle A-message from speeduino. This contains real time data.
void HandleA()
{
  Serial2.print ("A ");
  data_error = false;
  for (int i=0; i<75; i++) {
    SpeedyResponse[i] = Serial2.read();
    }
  processData();                  // do the necessary processing for received data
  //displayData();                  // only required for debugging
  doRequest = true;               // restart data reading
  oldtime = millis();             // zero the timeout
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}
//================================

// Handle R-message from speeduino. This is a request for some data.
void HandleR()
{
  Serial2.println ("R ");
  byte tmp0;
  byte tmp1;
  canin_channel = Serial2.read();
  tmp0 = Serial2.read();  // read in lsb of source can address
  tmp1 = Serial2.read();  // read in msb of source can address
  CanAddress = tmp1<<8 | tmp0 ;
  SendDataToSpeeduino();  // send the data to speeduino
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}
//================================

// Read the first byte from serial to determine what kind of message it is
void ReadSerial()
{
  currentCommand = Serial2.read();
  switch (currentCommand)
  {
    case 'A':  // Speeduino sends data in A-message
      SerialState = A_MESSAGE;
    break;
    case 'R':  // Speeduino requests data in R-message
      SerialState = R_MESSAGE;
    break;
    default:
      Serial.print ("Not an A or R message ");
      Serial.println (currentCommand);
    break;
  }
}
//================================

// Main loop
void loop() {
  unsigned long NowDashUpdate = millis();
  switch(SerialState) {
    case NOTHING_RECEIVED:
      if (Serial2.available() > 0) { ReadSerial(); }
      break;
    case A_MESSAGE:
      if (Serial2.available() >= 74) { HandleA(); }
      break;
    case R_MESSAGE:
      if (Serial2.available() >= 3) {  HandleR(); }
      break;
    case PWM_MESSAGE:
      // Not implemented, add if needed
      break;
    default:
      break;
  }
  if ( (millis()-oldtime) > 500) {
    oldtime = millis();
    Serial.println ("Timeout from speeduino!");
    doRequest = true;
  }
  readCanMessage();
  
  // Update dash values at defined rate 
  if (NowDashUpdate - lastDashUpdate > dashupdaterate) {
    lastDashUpdate = NowDashUpdate;
    updateDashValues();
  }
}
//================================
