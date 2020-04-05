/*
 * Lora GPS Tracker
 * Lucas Giebler
 * 2019-12-01
 * This is a GPS tracker that sends data over LoRa radio to another node for output.
 * It sends LoRa data to the project LoRa_GPS_Receiver code.
 *
 * Target Board is: "Adafruit M0 Feather" LoRa board or "Heltech WiFi LoRa 32" OLED board (see board target macros below)
 *
 *
 * Power management on feather: https://learn.adafruit.com/adafruit-feather-m0-adalogger/power-management
 * and: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/power-management
 *
 * RTC wake up: https://www.arduino.cc/en/Reference/RTC
 * example: https://www.arduino.cc/en/Tutorial/SleepRTCAlarm
 * Sleep mode example: https://github.com/patrickmoffitt/Adafruit-Feather-M0-Motion-Camera
 * Add Library RTCZero (no need to add URL, just search inlibrary manager)
 * Code for it: https://github.com/arduino-libraries/RTCZero/blob/master/src/RTCZero.cpp
 *
 * Radio Chip info: https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1276#download-resources
 * LoRa Radio info: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
 * RF95 header file: http://www.airspayce.com/mikem/arduino/RadioHead/RH__RF95_8h_source.html
 * Defauts:  after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol (7), CRC on
 * More info on LoRa: https://www.thethingsnetwork.org/forum/t/big-esp32-sx127x-topic-part-2/11973
 *
 * Spreading factor info. Heltec uses the number 11. For Dx info, See line 594 in above RF95 header link.
 #define RH_RF95_SPREADING_FACTOR_64CPS                0x60
 #define RH_RF95_SPREADING_FACTOR_128CPS               0x70
 #define RH_RF95_SPREADING_FACTOR_256CPS               0x80
 #define RH_RF95_SPREADING_FACTOR_512CPS               0x90
 #define RH_RF95_SPREADING_FACTOR_1024CPS              0xa0
 #define RH_RF95_SPREADING_FACTOR_2048CPS              0xb0
 #define RH_RF95_SPREADING_FACTOR_4096CPS              0xc0

 * BN-180 GPS/GNSS chip info: https://www.u-blox.com/en/product/ubx-m8030-series#tab-document-resources
 * General info on Adafruit GPS library: https://core-electronics.com.au/tutorials/how-to-use-gps-with-arduino.html
 * Also check out: Bolder Flight Systems UBLOX library.
 * And GPS Flora / FeatherWing example: GPS_HardwareSerial_Parsing
 */

#define myVERSION "1.03"

// how often to get GPS data and send via LoRa
#define GPS_UPDATE_RATE (5000)

// Set LoRa frequency, must match RX's freq!
#define RF95_FREQ 912.0

// set to 1 to enable waiting for remote LoRa device to send back after this transmits. Only for testing.
#define ENABLE_LORA_RX_ACK  0

// set to 1 to enable putting the LoRa radio in sleep mode after Tx.
#define ENABLE_LORA_SLEEP   1

// set to 1 to enable putting M0 (or ESP32) to sleep after LoRa Tx, wakes via RTC
#define ENABLE_MCU_SLEEP    1

// set to 1 to enable debug data
#define ENABEL_DEBUG_INFO   1

// define board target hardware macro (uncomment only one)
// Board: "adafruit feather M0" LoRa Board
#define FEATER_M0_LORA

// Heltech board with OLED
//#define HELTECH_OLED_LORA

// ------------------------------------------
#ifdef FEATER_M0_LORA
 /* for feather m0  */
#include <SPI.h>
#include <RH_RF95.h>  /* the LoRa RadioHead library from: adafruit link above */

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED_STATUS  (13)  /* LED on pin 13 */
#define VBATPIN A7      // vbatt analog input.


// Singleton instance of the radio driver
RH_RF95 LoRadio(RFM95_CS, RFM95_INT);

// ---  BEGIN  RTC for wake up ---
#if (ENABLE_MCU_SLEEP == 1)
#include <RTCZero.h>

/* Create an rtc object */
RTCZero rtc;  // declare the Real Time Clock object

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 00;
const byte hours = 17;

/* Change these values to set the current initial date */
const byte day = 17;
const byte month = 11;
const byte year = 15;
// ---  END  RTC for wake up ---
#endif  // #if (ENABLE_MCU_SLEEP == 1)

#endif  /* for feather m0  */
// ------------------------------------------

// adafruit GPS library
#include <Adafruit_GPS.h>
#include <stdlib.h>

// include for dtostrf()
//#include <stdlib_noniso.h>  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/stdlib_noniso.h


// BEGIN -- determine RAM remaining in M0 arduino board ----
// sbrk() should be declared in unistd.h but not on Adafruit M0:
extern "C" char *sbrk(int i);
// If this is already defined, then wrong board is selected:
// void* sbrk(ptrdiff_t)  void *  _EXFUN(sbrk,  (ptrdiff_t __incr));
int getFreeRam () {
  char stack_dummy = 0;
  return &stack_dummy - (char*)sbrk(0);
}
// END -- determine RAM remaining in M0 arduino board ----

// declare the GPS object. This object provides control the GPS module.
Adafruit_GPS GPS(&Serial1);

// --------------------------------------------------------------------
// the setup function runs once when you press reset or power the board
// --------------------------------------------------------------------
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  ledCode(2); // blink out info code

 // Serial is our command menu port
  Serial.begin(115200);

#if (0)
  // test our double to string converter
  char testBuff[16];
  double testd = 1.0001;
  Serial.print("1.0001 test: "); Serial.println(dtostrf(testd, 5,5, testBuff));
  testd = 1.002;
  Serial.print("1.002 test: "); Serial.println(dtostrf(testd, 5,5, testBuff));
  testd = 1.03;
  Serial.print("1.03 test: "); Serial.println(dtostrf(testd, 5,5, testBuff));
  testd = 1.4012;
  Serial.print("1.4012 test: "); Serial.println(dtostrf(testd, 4,4, testBuff));
  delay(1000);
#endif

  // now setup GPS serial port
  // BN-180 GPS is 9600

  //These lines configure the GPS Module
  if( !GPS.begin(9600) )
  {
    Serial.println("GPS Object begin() Failed");
    ledCode(5); // blink out info code
  }else
  {
#if (ENABLE_MCU_SLEEP == 0 && ENABEL_DEBUG_INFO == 1)
    Serial.println("GPS Object init OK");
#endif
  }
  // These commands do not seem to work on BN-180 device
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Sets output to only RMC and GGA sentences
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Sets the output to 1/second.
 // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // Sets the output to 5 second intervals

  // manual reset LoRa radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!LoRadio.init()) {
    Serial.println("LoRa radio init failed");
  //  Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ledCode(3); // blink out error code
  }
#if (ENABLE_MCU_SLEEP == 0 && ENABEL_DEBUG_INFO == 1)
  Serial.println("LoRa radio init OK!");
#endif
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!LoRadio.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ledCode(4); // blink out error code
  }
#if (ENABLE_MCU_SLEEP == 0 && ENABEL_DEBUG_INFO == 1)
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
#endif
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol (7), CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  LoRadio.setTxPower(23, false);

#if (ENABEL_DEBUG_INFO == 1)
  // dump registers
  Serial.println("+++ LoRa Radio Registers +++");
  LoRadio.printRegisters(); // only dumps regs: 0 to 0x27
  Serial.println("--- LoRa Radio Registers ---");
  ledCode(2); // blink out info code
  delay(2000);
#endif

// setup RTC to wake up device periodically
#if (ENABLE_MCU_SLEEP == 1)
  rtc.begin();

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  rtc.setAlarmTime(17, 00, 10);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  rtc.attachInterrupt(alarmMatch);

  rtc.standbyMode();
#endif
}

void alarmMatch()
{
  digitalWrite(LED_STATUS, HIGH);
}
// -----------------------------------------------
//        global variables:
// -----------------------------------------------
String gpsPacket;
char gpsPacket_[100] = "";
uint16_t packetnum = 0;  // packet counter, we increment per xmission
int8_t ledMode  = 1;    // 0 normal blink, 1 heartbeat blink, 3 LoRa Tx Blink
int8_t loRaState = 0;   // 0 Tx, 1 Rx, 2 wait ACK

bool gpsMonitor = 0;    // 0 = disabled, 1 = enabled. Send GPS data out serial port
int gpsMonitorRaw = 0; // 0 = use GPS object. 1 = direct read from Serial1, 2 = all raw data from Serial1.
bool loraMonitor = 0;   //

// the loop function runs over and over again forever
// make loop more modular: https://arduino.stackexchange.com/a/37748
void loop() {
  int fix;
  menuCommand();
  fix = getGPS(gpsPacket_);
  // transmit any GPS info
  txLoRa(gpsPacket_);
  // wait a short time for any command from remote
  rxLoRa();
#if (ENABLE_MCU_SLEEP == 1)
  // If GPS has fix, then go to sleep for a while before sending next fix.
  digitalWrite(LED_STATUS, LOW);
#else
  // if get here, then system does not have a GPS fix and did not sleep.
  ledControl();
#endif
}

/**
 * float to string converter. 6 digits.
 * provide string buffer of at least 12 chars.
 */
char* dtostrf(double df, int minln, int maxdig, char* in_buff)
{
  char tp_buff[16];
  if( maxdig > 8 )
    maxdig = 8;
  in_buff[0] = 0;
  // convert integer part
  int whole = (int) df;
  itoa(whole, in_buff, 10);
  strcat(in_buff, ".");
  // convert fractional part
  df = df - (double) whole;
  if(whole < 0)
    df *= -1.0;
  long frac = (long) (df * pow(10.0,(double)maxdig));
  ltoa(frac, tp_buff, 10);
  // pad with 0's to left
  int zp = maxdig - strlen(tp_buff);
  while(zp > 0)
  {
    strcat(in_buff,"0");
    zp--;
  }
  strcat(in_buff, tp_buff);
  return in_buff;
}

/** *******************************************************
 * @brief define our command menu state machine states.
 **********************************************************/
typedef enum{
  menuNop,      // do nothing
  menu,         // output a menu of commands
  menuWait,     // wait for menu command
} cmdStates;
static cmdStates cmdState = menu;
const char commandMenu[] = {
"\n####################################################################\n"
  "#  GPS Tracker Transmitter "myVERSION"                                    #\n"
//  "#  Author: Lucas Giebler                                           #\n"
  "#  Command menu:                                                   #\n"
  "#    0  - show this menu and disable all monitors                  #\n"
  "#    1  - enable/disable monitor of GPS data to serial port        #\n"
  "#    2  - enable/disable monitor of LoRa data to serial port       #\n"
  "#    3  - toggle LED blink modes                                   #\n"
  "#    4  - cycle Raw GPS details (Off|Low|High)                     #\n"
  "#                                                                  #\n"
  "####################################################################"
  };
  // manually coordinate values with menu above.
#define MENU_USAGE  0
#define MENU_TOGGLE_MONITOR_GPS 1
#define MENU_TOGGLE_MONITOR_LORA 2
#define MENU_TOGGLE_LED_MODES 3
#define MENU_TOGGLE_GPS_RAW 4
// ***************************************************************
// menuCommand()
// Handles the command menu via the USB serial port
// Called from the main loop.
// This is a cooperative multi-task designed function
// ***************************************************************
void menuCommand()
{
  if( cmdState == menuNop )
  {
    return;
  }else if ( cmdState == menu )
  {
    // --------------------------------------
    // ----  display menu command        ----
    Serial.println(commandMenu);

    Serial.print("# Input command : ");
    cmdState = menuWait;
  }else if ( cmdState == menuWait && Serial.available())
  {
    // --------------------------------------
    // ----  wait for menu command input ----
    String cmd = Serial.readStringUntil('\n');
    Serial.print(cmd);
    int cmdNmbr = -9999;
    sscanf(cmd.c_str(), "%d", &cmdNmbr);
    switch(cmdNmbr)
    {
      case MENU_USAGE:   // show main menu
        cmdState = menu;
        break;
      case MENU_TOGGLE_MONITOR_GPS:
        gpsMonitor ^= 1;  // toggle flag
        Serial.print(" - GPS Monitor: ");Serial.println(gpsMonitor?"ON":"OFF");
        break;
      case MENU_TOGGLE_MONITOR_LORA:
        loraMonitor ^= 1;
        Serial.print(" - LoRa Monitor: ");Serial.println(loraMonitor?"ON":"OFF");
        break;
      case MENU_TOGGLE_LED_MODES:           // LED blink modes
        ledMode ^= 1;
        Serial.print(" - LED Mode: ");Serial.println(ledMode?"HeartBeat":"Normal");
        break;
      case MENU_TOGGLE_GPS_RAW:
        gpsMonitorRaw++;
        gpsMonitorRaw &= 0x03;  // cycle through 4 modes.
        if(gpsMonitorRaw>0) gpsMonitor=1; // turn on general GPS montor mode if needed.
        Serial.print(" - GPS Raw Monitor: ");
        Serial.print(gpsMonitorRaw?"ON-":"OFF");
        Serial.println(gpsMonitorRaw==1?"Low":gpsMonitorRaw>1?"High":"");
        break;
      default:
      case -9999:
        Serial.println(" - invalid command");
        cmdState = menu;
        break;
    }
  }
}
/** *********************************************************************************
 * getGPS gets the location data from the GPS receiver and makes a LoRa ready packet.
 * Returns a string of GPS data in the passed in char buffer.
 * Make sure char buffer is large enough.
 * Called from the main loop.
 * This is a cooperative multi-task designed function.
 *
 * Returns 1 if GPS has fix, 0 if not
 *
 * Section numbers are for u-blox 8-M8 recevier protocol spec.
 *
 * Look for one these types of lines from receiver:
 * $GNGGA,010648.00,5554.47138,N,08952.82014,W,1,12,0.64,301.9,M,-31.3,M,,*66
 *  See section 31.2.4
 * $GNRMC,010647.00,A,5554.47155,N,08952.82016,W,0.526,,301119,,,A*64
 *  See section 31.2.14  This one has ground speed and direction vector.
 * $GNGLL,5554.47155,N,08952.82016,W,010647.00,A,A*73
 *  See section 31.2.5
 *
 *  Our LoRa packet format:
 *  $,<GPS Fix 0:1>,<Lat Deg>,<Long Deg>,<Vector speed>,<Vector angle>,<Altitude>,<Sat Count>,<V Battery>,<packet count>
 ********************************************************************************** */
int getGPS(char* gpsPacket_)
{
  const char message[]="$GNRMC";

  static unsigned long lastTime = 0;
  const long interval = GPS_UPDATE_RATE;
  unsigned long nowTk = millis();
  int fix = GPS.fix;

  if( gpsMonitorRaw && Serial1.available() > 0 )
  {
    String gps = Serial1.readStringUntil('\n');
    // find string starting with message
    String tag = gps.substring(0, gps.indexOf(','));
    if( tag == message)
    {
      gpsPacket = gps.substring(gps.indexOf(',')+1);
      if( gpsMonitor && gpsMonitorRaw == 1)
      {
        Serial.print("GPS Found it! ");
        Serial.print("GPS packet [");Serial.print(gpsPacket.c_str());Serial.println("]");
      }
    }
    if( gpsMonitor && gpsMonitorRaw > 1)
    {
        Serial.println(gps.c_str());
    }
  }else if(gpsMonitorRaw == 0){
    while( GPS.available() > 0 ){
      char c = GPS.read();
      if(gpsMonitor){
//        Serial.print(c);
      }
    }
    // use GPS object
    if( GPS.newNMEAreceived() ) //This will return a boolean TRUE/FALSE depending on the case.
    {
      // check if desired message
      GPS.parse(GPS.lastNMEA()); //This is going to parse the last NMEA sentence the Arduino has received, breaking it down into its constituent parts.
//      String gps = String(GPS.lastNMEA());
//      String tag = gps.substring(0, gps.indexOf(','));
//      if( tag == message)
//      {
//        if(gpsMonitor && (nowTk - lastTime) > interval){
//        }
//      }// if( tag == message)
    }
  }
  // copy GPS data to LoRa packet if delay time elapsed.
  if ( nowTk - lastTime > interval ) {
    lastTime = nowTk;
    char buff[20];
    // LoRa format is:
    // $<fix>,<lat>,<lon>,<speed>,<dir>,<altitude>,<#sats>,<batteryV>,<pkt-count>
    strcpy(gpsPacket_, "$");
    itoa((int)GPS.fix, buff, 10);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    dtostrf((double)GPS.latitudeDegrees, 7, 7, buff);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    dtostrf(GPS.longitudeDegrees, 7, 7, buff);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    dtostrf(GPS.speed, 2, 2, buff);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    dtostrf(GPS.angle, 2, 2, buff);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    dtostrf(GPS.altitude, 2, 2, buff);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    itoa((int)GPS.satellites, buff, 10);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    //  get battery Voltage
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    dtostrf(measuredvbat, 2, 2, buff);
    strcat(gpsPacket_, buff);
    strcat(gpsPacket_,",");
    itoa((int)packetnum++, buff, 10);
    strcat(gpsPacket_, buff);

    if( gpsMonitor )
    {
      Serial.print("\nGPS packet [");Serial.print(gpsPacket_);Serial.println("]");
        //Print the current date/time/etc
      Serial.print("Time: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

        //If GPS module has a fix, line by line prints the GPS information
      if (fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 5); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 5); Serial.println(GPS.lon);
        Serial.print("Location (in degrees, works with Google Maps): ");
        Serial.print(GPS.latitudeDegrees, 8);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees, 8);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
    }
  }else
  {
    strcpy(gpsPacket_, "");
  }
  return fix;
}

// -----------------------------------------------------------------------
// Send a packet of bytes over the LoRa radio
// txPacket is null terminated string, null is not transmotted.
//
// Called from the main loop.
// This is a cooperative multi-task designed function
// -----------------------------------------------------------------------
void txLoRa(char* txPacket) {
  static unsigned long lastTime = 0;
  const long interval = 3000;

  // if no packet, no send needed.
  if(!txPacket || txPacket[0] == 0) return;

  unsigned long now = millis();

  if ( now - lastTime > interval && loRaState == 0 ) {
    // LoRa state is Tx
    lastTime = now;
    if( loraMonitor )
    {
      Serial.print("LoRa Sending ["); Serial.print(txPacket);Serial.println("]");
      delay(10);
    }
    LoRadio.send((uint8_t *)txPacket, strlen(txPacket));
    if( loraMonitor )
    {
      Serial.println("Waiting for packet to complete...");
      delay(10);
    }
    LoRadio.waitPacketSent();
    // put radio to sleep:
#if (ENABLE_LORA_SLEEP == 1 && ENABLE_LORA_RX_ACK == 0)
    LoRadio.sleep();
#endif
#if (ENABLE_LORA_RX_ACK == 1)
    // move to Rx state.
    loRaState = 1;
#endif
  }
}

#define LORA_RX_WAIT  (20)    // number of 10 mSec intervals to wait for Rx data.
// --------------------------------------------------------------------
// Receive data from the LoRa radio
// Needs to be called repeatedly while waiting for response.
// Called from the main loop.
// This is a cooperative multi-task designed function
// --------------------------------------------------------------------
void rxLoRa()
{

#if (ENABLE_LORA_RX_ACK == 1)   // wait for RX only if enabled.
  static uint8_t tm_retries = LORA_RX_WAIT;    // number of 10 mSec intervals to wait for Rx data.
  static unsigned long lastTime = 0;
  const long interval = 10;
  unsigned long now = millis();

  if ( now - lastTime > interval && loRaState >= 1 ) {
    lastTime = now;

    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if( loRaState == 1 )
    {
      // LoRa Rx state.
      // Move to Wait LoRa state.
      loRaState++;
      if( loraMonitor )
      {
        Serial.println("Waiting for reply...");
      }
    }
    if (LoRadio.waitAvailableTimeout(10))
    {
      // Should be a reply message for us now
      if (LoRadio.recv(buf, &len))
      {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(LoRadio.lastRssi(), DEC);
      }
      else
      {
        Serial.println("Receive failed");
      }
      // move to Tx state.
      loRaState = 0;
#if (ENABLE_LORA_SLEEP == 1 )
      LoRadio.sleep();
#endif

    }
    else if(--tm_retries == 0)
    {
      if( loraMonitor )
      {
        Serial.println("No reply, is there a listener around?");
      }
      // move to Tx state.
      loRaState = 0;
    }
  }
  if( loRaState == 0 )
  {
    // Tx state, reset RX timeout retries.
    tm_retries = LORA_RX_WAIT;
  }

#else
  // waiting for Rx is disable, therefore
  // move to Tx state.
  loRaState = 0;
#endif  // #if (ENABLE_LORA_RX_ACK == 1)

}


#define LED_BLINK_FAST  (100)
#define LED_BLINK_      (500)
// --------------------------------------------------------------------
// led blink control
// Call periodically from main loop.
// This is a cooperative multi-task designed function
// uses global: int8_t ledMode  = 0;    // 0 normal blink, 1 heartbeat blink.
// --------------------------------------------------------------------
void ledControl() {

  static unsigned long lastTime = 0;
  static unsigned long interval = LED_BLINK_FAST;
  static int state = 0;

  unsigned long tm_now = millis();
  long elapsed = tm_now - lastTime;

// heart beat blinking
  if ( (elapsed) >= interval && (state == 0 || state == 2)) {
    state++;
    lastTime = tm_now;
    digitalWrite(LED_STATUS, HIGH);
    interval = LED_BLINK_FAST;
  }else if ( elapsed >= interval && (state == 1 || state == 3)) {
    state++;
    lastTime = tm_now;
    digitalWrite(LED_STATUS, LOW);
    if( state > 3 && ledMode == 1)
    {
      // end of heartbeat 2 blinks, so off for longer time.
      interval = LED_BLINK_;
    }else{
      interval = LED_BLINK_FAST;
    }
    state &= 0x03;  // ensure 4 states
  }
}

// blink out code, then pause
// call again to repeat
void ledCode(int eCode)
{
  digitalWrite(LED_STATUS, LOW);
  delay(500);
  for(int i = 0; i< eCode; i++)
  {
    digitalWrite(LED_STATUS, HIGH);
    delay(250);
    digitalWrite(LED_STATUS, LOW);
    delay(250);
  }
  delay(500);
}
// -------------------------------------------------------------------
//  End of file
// -------------------------------------------------------------------
