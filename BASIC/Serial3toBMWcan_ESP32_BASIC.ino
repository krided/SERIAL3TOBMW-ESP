// ESP32 version of Serial3toBMWcan using MCP_CAN library
// This code reads real time data from Speeduino EFI using serial connection and converts that to CAN messages for BMW e39/e46 instrument clusters
// Hardware: ESP32 WROOM with MCP2515 CAN controller
// Converted from STM32 version by pazi88

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

#include <SPI.h>
#include <mcp_can.h>
#include <HardwareSerial.h>
#include <Ticker.h>

// Pin definitions for ESP32
#define SERIAL1_RX_PIN 22 // GPIO22 (Serial1 RX)
#define SERIAL1_TX_PIN 23 // GPIO23 (Serial1 TX)

// ==================== TEMPERATURE LIGHT ====================
#define HOT_TEMP_BLINK 100    // Set temperature for overheat light to start blinking
#define HOT_TEMP_SOLID 120    // Set temperature for overheat light to stay solid
#define BLINKING 1            // 1 = enable blinking, 0 = solid light only

// MCP2515 SPI pins
#define MCP2515_CS_PIN   5  // GPIO5 (SPI CS)
#define MCP2515_INT_PIN  4  // GPIO4 (MCP2515 INT)
#define MCP2515_MOSI_PIN 21 // GPIO21 (SPI MOSI) - custom pin to avoid conflict with Serial1
#define MCP2515_MISO_PIN 19 // GPIO19 (SPI MISO)
#define MCP2515_SCK_PIN  18 // GPIO18 (SPI SCK)

//Bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))

#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define A_MESSAGE               2
#define PWM_MESSAGE             3

//Stock M52TU injectors scaling. Adjust this to trim fuel consumption. (injector size affects this)
//Bosch 0280155831 19000
#define PW_ADJUST           25000

#define SerialUpdateRate 30   // 30 Hz rate limit to update secondary serial data from speeduino
#define PWMFanFrequency 100   // 100 Hz Frequency for the BMW PWM fan
#define ClusterUpdateRate 50  // 50 Hz Frequency for the cars instrument cluster

#define BIT_LV_REQ_TCO_L       5  // Request For Lowering Cooling Temp (c_tco_bol_ect)
#define BIT_LV_ACCIN           6  // Air Conditioning Compressor Status (0=off, 1=on)
#define BIT_LV_ACIN            7  // Air Conditioning Request (0=off, 1=on)

// MCP_CAN instance
MCP_CAN CAN(MCP2515_CS_PIN);

// Create Serial1 instance for ESP32
HardwareSerial SpeeduinoSerial(1);

// ESP32 Timer variables
Ticker requestTicker;
Ticker sendTicker;
const uint8_t CLT_TEMP_OFFSET = 40; // Offset for CLT temperature conversion
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
uint8_t SpeedyResponse[100]; //The data buffer for the serial data. This is longer than needed, just in case
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
bool doRequest; // when true, it's ok to request more data from speeduino serial
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

// CAN message buffers
unsigned char CAN_msg_RPM[8];
unsigned char CAN_msg_CLT_TPS[8];
unsigned char CAN_msg_MPG_CEL[8];

// Timer interrupt functions
void onRequestTimer() {
  if (doRequest){
    SpeeduinoSerial.write("A"); // Send A to request real time data
    doRequest = false;
  }
  
  // The PWM fan duty is only requested once in a second. This doesn't need to be that frequently updated.
  if (rRequestCounter >= SerialUpdateRate){
    SpeeduinoSerial.write("r");  // Send r to request PWM fan duty
    SpeeduinoSerial.write(0xAA); // $tsCanId placeholder
    SpeeduinoSerial.write(0x30); // Send output channels command 0x30
    SpeeduinoSerial.write(123);  // LSB offset for receiving PWM fan duty. (202207 and older speeduino FWs use 121)
    SpeeduinoSerial.write(0);    // MSB offset for receiving PWM fan duty.
    SpeeduinoSerial.write(1);    // LSB length for receiving PWM fan duty.
    SpeeduinoSerial.write(0);    // MSB length for receiving PWM fan duty.
    rRequestCounter = 0;
  }
  else {
    rRequestCounter++;
  }
}

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

void onSendTimer() {
  // Send CAN messages in 50Hz phase from timer interrupt. This is important to be high enough Hz rate to make cluster work smoothly.
  if (ascMSG) {
    CAN_msg_RPM[0]= 0x05;
  }
  else {
    CAN_msg_RPM[0]= 0x01;
  }	
  CAN_msg_RPM[2]= rpmLSB; // RPM LSB
  CAN_msg_RPM[3]= rpmMSB; // RPM MSB
  // Send RPM message
  byte result = CAN.sendMsgBuf(0x316, 0, 8, CAN_msg_RPM);
  if (result == CAN_OK) {
  }
  // Send fuel consumption and error lights
  if (CEL < 200){  
    if (CEL < 100){
      CAN_msg_MPG_CEL[0]= 0x12;  // keep CEL And EML on for about 2 seconds
    }
    else{
      CAN_msg_MPG_CEL[0]= 0x02;  // keep CEL on for about 4 seconds
      }
    CEL++;
    }
  else{
    CAN_msg_MPG_CEL[0]= 0x00;  // CEL off
  }
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
  CAN_msg_MPG_CEL[1]= pwLSB;  // LSB Fuel consumption
  CAN_msg_MPG_CEL[2]= pwMSB;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL[3]= tempLight ;  // Overheat light
  CAN.sendMsgBuf(0x545, 0, 8, CAN_msg_MPG_CEL);
  
  //Send CLT and TPS
  CAN_msg_CLT_TPS[1]= CLT; // Coolant temp
  CAN_msg_CLT_TPS[5]= TPS; // TPS value.

  //Multiplexed Information in byte0
  switch (multiplex) {
  case 0: //CAN_LEVEL
    CAN_msg_CLT_TPS[0]= 0x11;
    break;
  case 1: //OBD_STEUER
    if (currentStatus.RPM < 400)
    {
      CAN_msg_CLT_TPS[0]= 0x80;
    }
    else
    {
      CAN_msg_CLT_TPS[0]= 0x86;
    }
    break;
  case 2: //MD_NORM
    CAN_msg_CLT_TPS[0]= 0xD9;
    break;
  default:
    CAN_msg_CLT_TPS[0]= 0x11;
    break;
  }
  CAN.sendMsgBuf(0x329, 0, 8, CAN_msg_CLT_TPS);

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

void readCanMessage(unsigned long canId, unsigned char len, unsigned char *buf) {
  switch (canId)
  {
    case 0x613:
      odometerLSB = buf[0];
      odometerMSB = buf[1];
      //Serial.print ("Odometer: "); Serial.println (odometerLSB + (odometerMSB << 8));
      FuelLevel = buf[2];
      //Serial.print ("Fuel level: "); Serial.println (FuelLevel);
      runningClock = ((buf[4] << 8) | (buf[3]));
    break;
    case 0x615:
      ambientTemp = buf[3];
      //Serial.print ("Outside temp: "); Serial.println (ambientTemp);
      acBitfield = buf[0];
      acBitfield2 = buf[4];
      eFanBitfield = buf[1];
    break;
    case  0x153: 
      ascMSG = true;
      VSS = ((buf[2] << 8) | (buf[1]));
      // conversion (speeduino doesn't have internal conversion for CAN data, so we do it here)
      VSS = VSS >> 7; // divide by 128
      VSS = VSS - 2;
    break;
    case  0x1F0:
      VSS1 = ((buf[1] << 8) | (buf[0]));
      // conversion
      VSS1 = VSS1 >> 4; // divide by 16
      VSS1 = VSS1 - 2;
      VSS2 = ((buf[3] << 8) | (buf[2]));
      VSS2 = VSS2 >> 4;
      VSS2 = VSS2 - 2;
      VSS3 = ((buf[5] << 8) | (buf[4]));
      VSS3 = VSS3 >> 4;
      VSS3 = VSS3 - 2;
      VSS4 = ((buf[7] << 8) | (buf[6]));
      VSS4 = VSS4 >> 4;
      VSS4 = VSS4 - 2;
    break;
    default:
      // nothing to do here
    break;
  }
}

void SendDataToSpeeduino(){
  SpeeduinoSerial.write("G");                      // reply "G" cmd
  switch (CanAddress)
  {
    case 0x613:  // Odometer and fuel level
      SpeeduinoSerial.write(1);                        // send 1 to confirm cmd received and valid
      SpeeduinoSerial.write(canin_channel);            // confirms the destination channel
      SpeeduinoSerial.write(odometerLSB);              // write back the requested data
      SpeeduinoSerial.write(odometerMSB);
      SpeeduinoSerial.write(FuelLevel);
      SpeeduinoSerial.write(lowByte(runningClock));
      SpeeduinoSerial.write(highByte(runningClock));
      for (int i=0; i<3; i++) {                // Rest will be zero
        SpeeduinoSerial.write(0);
      }
    break;
    case 0x615:  // Ambient temp
      SpeeduinoSerial.write(1);                        // send 1 to confirm cmd received and valid
      SpeeduinoSerial.write(canin_channel);            // confirms the destination channel
      for (int i=0; i<3; i++) {
          SpeeduinoSerial.write(0);
      }
      SpeeduinoSerial.write(ambientTemp);              // write back the requested data
        for (int i=0; i<4; i++) {
          SpeeduinoSerial.write(0);
      }
    break;
    case  0x153:  // VSS
      SpeeduinoSerial.write(1);                        // send 1 to confirm cmd received and valid
      SpeeduinoSerial.write(canin_channel);            // confirms the destination channel
      SpeeduinoSerial.write(0);
      SpeeduinoSerial.write(lowByte(VSS));
      SpeeduinoSerial.write(highByte(VSS));
      for (int i=0; i<5; i++) {
          SpeeduinoSerial.write(0);
      }
    break;
    case  0x1F0:  // VSS for each invidual wheel
      SpeeduinoSerial.write(1);                        // send 1 to confirm cmd received and valid
      SpeeduinoSerial.write(canin_channel);            // confirms the destination channel               //write back the requested data
      SpeeduinoSerial.write(lowByte(VSS1));
      SpeeduinoSerial.write(highByte(VSS1));
      SpeeduinoSerial.write(lowByte(VSS2));
      SpeeduinoSerial.write(highByte(VSS2));
      SpeeduinoSerial.write(lowByte(VSS3));
      SpeeduinoSerial.write(highByte(VSS3));
      SpeeduinoSerial.write(lowByte(VSS4));
      SpeeduinoSerial.write(highByte(VSS4));
    break;
    default:
      SpeeduinoSerial.write(0);                        // send 0 to confirm cmd received but not valid
      SpeeduinoSerial.write(canin_channel);            // destination channel
      for (int i=0; i<8; i++) {                // we need to still write some crap as an response, or real time data reading will slow down significantly
          SpeeduinoSerial.write(0);
      }
      Serial.print ("Wrong CAN address");
    break;
  }
}

// display the needed values in serial monitor for debugging
void displayData(){
  Serial.print ("RPM-"); Serial.print (currentStatus.RPM); Serial.print("\t");
  Serial.print ("PW-"); Serial.print (currentStatus.PW1); Serial.print("\t");
  Serial.print ("PWcount-"); Serial.print (PWcount); Serial.print("\t");
  Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
  Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");
}

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

void HandleA()
{
  data_error = false;
  for (int i=0; i<75; i++) {
    SpeedyResponse[i] = SpeeduinoSerial.read();
    }
  processData();                  // do the necessary processing for received data
  //displayData();                  // only required for debugging
  doRequest = true;               // restart data reading
  oldtime = millis();             // zero the timeout
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void HandleR()
{
  Serial.println ("R ");
  byte tmp0;
  byte tmp1;
  canin_channel = SpeeduinoSerial.read();
  tmp0 = SpeeduinoSerial.read();  // read in lsb of source can address
  tmp1 = SpeeduinoSerial.read();  // read in msb of source can address
  CanAddress = tmp1<<8 | tmp0 ;
  SendDataToSpeeduino();  // send the data to speeduino
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void Handle_r()
{
  uint8_t tmp_duty;
  Serial.println ("r ");
  data_error = false;
  if (SpeeduinoSerial.read() != 0x30){  // r-command always returns 0x30. Something is wrong if it doesn't.
    data_error = true;
  }
  else {
    tmp_duty = SpeeduinoSerial.read(); // read the fan duty.
    tmp_duty = tmp_duty / 2;   // speeduino range is 0-200. Change it to 0-100
    if (tmp_duty < 10){        // adjust the range to valid 10-90 for BMW fan
      tmp_duty = 10;
    }
    else if (tmp_duty > 90){
      tmp_duty = 90;
    }
    PWMfanDuty = tmp_duty;
    // Note: ESP32 PWM implementation would need to be added here if needed
  }
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void ReadSerial()
{
  currentCommand = SpeeduinoSerial.read();
  switch (currentCommand)
  {
    case 'A':  // Speeduino sends data in A-message
      SerialState = A_MESSAGE;
    break;
    case 'R':  // Speeduino requests data in R-message
      SerialState = R_MESSAGE;
    break;
    case 'r':  // Speeduino sends the Fan PWM duty in data in r-message, because we requested it.
      SerialState = PWM_MESSAGE;
    break;
    default:
      Serial.print ("Not an A or R message ");
      Serial.println (currentCommand);
    break;
  }
}

void setup(){
  
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("ESP32 BMW CAN Bridge starting...");
  
  // Configure custom SPI pins for MCP2515
  SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);
  
  // Initialize Serial1 with custom pins for Speeduino communication
  SpeeduinoSerial.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  
  // Initialize MCP2515 using MCP_CAN library
  byte result = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  if (result == CAN_OK) {
    Serial.println("MCP2515 initialized successfully");
    CAN.setMode(MCP_NORMAL);
    Serial.println("MCP2515 set to normal mode");
  } else {
    Serial.println("Failed to initialize MCP2515 - check wiring and crystal frequency");
  }
  
  doRequest = false;
  rRequestCounter = 0;

  // Initialize static CAN message data
  // send this message to get rid of EML light and also set the static values for the message
  CAN_msg_MPG_CEL[0]= 0x02;  // error State
  CAN_msg_MPG_CEL[1]= 0x00;  // LSB Fuel consumption
  CAN_msg_MPG_CEL[2]= 0x00;  // MSB Fuel Consumption
  CAN_msg_MPG_CEL[3]= 0x00;  // Overheat light
  CAN_msg_MPG_CEL[4]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL[5]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL[6]= 0x00; // not used, but set to zero just in case.
  CAN_msg_MPG_CEL[7]= 0x00; // not used, but set to zero just in case.

  // set the static values for the other two messages
  CAN_msg_RPM[0]= 0x01;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
  CAN_msg_RPM[1]= 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation!!
  CAN_msg_RPM[4]= 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
  CAN_msg_RPM[5]= 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
  CAN_msg_RPM[6]= 0x00;  //not used
  CAN_msg_RPM[7]= 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention

  CAN_msg_CLT_TPS[0]= 0x11;  //Multiplexed Information
  CAN_msg_CLT_TPS[2]= 0xB2;  //CLT temp
  CAN_msg_CLT_TPS[3]= 0x00;  //Baro
  CAN_msg_CLT_TPS[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_CLT_TPS[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_CLT_TPS[7]= 0x00;  //not used, but set to zero just in case.

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

  // Setup ESP32 periodic tasks using Ticker
  requestTicker.attach(1.0 / SerialUpdateRate, onRequestTimer); // 30Hz
  sendTicker.attach(1.0 / ClusterUpdateRate, onSendTimer);      // 50Hz

  Serial.println ("Version date: 20.8.2025 - ESP32 Version with MCP_CAN"); // To see from debug serial when is used code created.
  doRequest = true; // all set. Start requesting data from speeduino
  rRequestCounter = SerialUpdateRate;
  
  Serial.println("ESP32 BMW CAN Bridge initialized successfully!");
}

// main loop
void loop() {
  switch(SerialState) {
    case NOTHING_RECEIVED:
      if (SpeeduinoSerial.available() > 0) { ReadSerial(); }  // read bytes from serial to define what message speeduino is sending.
      break;
    case A_MESSAGE:
      if (SpeeduinoSerial.available() >= 74) { HandleA(); }  // read and process the A-message from serial, when it's fully received.
      break;
    case R_MESSAGE:
      if (SpeeduinoSerial.available() >= 3) {  HandleR(); }  // read and process the R-message from serial, when it's fully received.
      break;
    case PWM_MESSAGE:
      if (SpeeduinoSerial.available() >= 2) {  Handle_r(); }  // read and process the r-message from serial, when it's fully received.
      break;
    default:
      break;
  }

  if ( (millis()-oldtime) > 500) { // timeout if for some reason reading serial fails
    oldtime = millis();
    Serial.println ("Timeout from speeduino!");
    doRequest = true;                // restart data reading
  }
  
  // Read incoming CAN messages from instrument cluster
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId;
  
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&canId, &len, buf);
    readCanMessage(canId, len, buf);
  }
}
