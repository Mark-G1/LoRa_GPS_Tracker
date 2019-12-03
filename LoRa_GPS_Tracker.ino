/*
 * Lora GPS Tracker
 * Lucas Giebler
 * 2019-12-01
 * This is a GPS tracker that sends data over LoRa radio to another node for output.
 * Target hardware is: Adafruit M0 Feather LoRa board or Heltech LoRa OLED board (see board target macros below)
 * 
 * LoRa Radio info: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
 * 
 */

#define myVERSION "1.00"

// define board target hardware macro (uncomment only one)
// adafruit feather M0 LoRa Board
#define FEATER_M0_LORA

// Heltech board with OLED
//#define HELTECH_OLED_LORA

#ifdef FEATER_M0_LORA
 /* for feather m0  */
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Singleton instance of the radio driver
RH_RF95 LoRadio(RFM95_CS, RFM95_INT);

#endif  /* for feather m0  */

#define LED_STATUS  (13)  /* LED on pin 13 */

#define GPS_UPDATE_RATE (5000)


// Set frequency, must match RX's freq!
#define RF95_FREQ 912.0

// BEGIN -- determine RAM remaiing in M0 arduino board ----
extern "C" char *sbrk(int i);

int getFreeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
// END -- determine RAM remaiing in M0 arduino board ----

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(LED_STATUS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

 // Serial is our command menu port
  Serial.begin(230400);
  while (!Serial) {
    delay(1);
  }
  delay(2000);
  // now setup GPS serial port 
  // BN-180 GPS is 9600

  Serial1.begin(9600);
  while(!Serial1) {
    delay(1);
  }

  Serial.println("LoRa TX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!LoRadio.init()) {
    Serial.println("LoRa radio init failed");
  //  Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!LoRadio.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  LoRadio.setTxPower(23, false);

}

// -----------------------------------------------
//        global vairables:
// -----------------------------------------------
String gpsPacket;
char gpsPacket_[20] = "Hello World #      ";

uint16_t packetnum = 0;  // packet counter, we increment per xmission
int8_t ledMode  = 1;    // 0 normal blink, 1 heartbeat blink, 3 LoRa Tx Blink
int8_t loRaState = 0;   // 0 Tx, 1 Rx

bool gpsMonitor = 0;    // 0 = disabled, 1 = enabled. Send GPS data out serial port
bool loraMonitor = 0;   // 

// the loop function runs over and over again forever
// make loop more modular: https://arduino.stackexchange.com/a/37748
void loop() {
  menuCommand();
  ledControl();
  getGPS(gpsPacket_);
  txLoRa(gpsPacket_);
  rxLoRa();
}

/**
 * @brief define our command menu state machine states.
 */
typedef enum{
  menuNop,      // do nothing
  menu,         // output a menu of commands
  menuWait,     // wait for menu command
} cmdStates;
static cmdStates cmdState = menu;
const char commandMenu[] = {
"\n####################################################################\n"
  "#  GPS Tracker Transmitter "myVERSION"                                    #\n"
  "#  Author: Lucas Giebler                                           #\n"
  "#  Command menu:                                                   #\n"
  "#    0  - show this menu and disable all monitors                  #\n"
  "#    1  - enable/disable monitor of GPS data to serial port        #\n"
  "#    2  - enable/disable monitor of LoRa data to serial port       #\n"
  "#    3  - toggle LED blink modes                                   #\n"
  "#                                                                  #\n"
  "####################################################################"
  };

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
      case 0:   // show main menu
        cmdState = menu;
        break;
      case 1:
        gpsMonitor ^= 1;  // toggle flag
        Serial.print(" - GPS Monitor: ");Serial.println(gpsMonitor?"ON":"OFF");
        break;
      case 2:
        loraMonitor ^= 1;
        Serial.print(" - LoRa Monitor: ");Serial.println(loraMonitor?"ON":"OFF");
      break;
      case 3:           // LED blink modes
        ledMode ^= 1;
        Serial.print(" - LED Mode: ");Serial.println(ledMode?"HeartBeat":"Normal");
      break;
      default:
      case -9999:
      Serial.println(" - invalid command");
      cmdState = menu;
      break;
    }
  }
}
/*
 * getGPS gets the location data from the GPS recevier and makes a LoRa ready packet.
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
 */
void getGPS(char* gpsPacket_)
{
  const char message[]="$GNRMC";
  
  static unsigned long lastTime = 0;
  const long interval = GPS_UPDATE_RATE;
  unsigned long now = millis();

  if( Serial1.available() > 0 )
  {
    String gps = Serial1.readStringUntil('\n');
    // find string starting with message
    String tag = gps.substring(0, gps.indexOf(','));
    if( tag == message)
    {
      gpsPacket = gps.substring(gps.indexOf(',')+1);
      if( gpsMonitor )
      {
        Serial.print("GPS Found it! ");
        Serial.print("GPS packet [");Serial.print(gpsPacket.c_str());Serial.println("]");
      }
    }
  }
  
  if ( now - lastTime > interval ) {
    lastTime = now;
    gpsPacket_[0] = 'H';  // put the H back in.
    itoa(packetnum++, gpsPacket_+13, 10);
    gpsPacket_[19] = 0;
    if( gpsMonitor )
    {
      Serial.print("GPS packet [");Serial.print(gpsPacket_);Serial.println("]");
    }
  }else
  {
    gpsPacket_[0] = 0;
  }
  
}


void txLoRa(char* txPacket) {
  static unsigned long lastTime = 0;
  const long interval = 3000;

  // if no packet, no send needed.
  if(!txPacket || txPacket[0] == 0) return;
  
  unsigned long now = millis();

  if ( now - lastTime > interval && loRaState == 0 ) {
    lastTime = now;
    if( loraMonitor )
    {
      Serial.print("LoRa Sending ["); Serial.print(txPacket);Serial.println("]");
      delay(10);
    }
    LoRadio.send((uint8_t *)txPacket, 20);
    if( loraMonitor )
    {
      Serial.println("Waiting for packet to complete..."); 
      delay(10);
    }
    LoRadio.waitPacketSent();
    // move to Rx state.
    loRaState = 1;
  }
}

void rxLoRa()
{
  static uint8_t tm_retries = 40;    // number of 20 mSec intervals to wait for Rx data.
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
      loRaState = 0;
    }
    else if(--tm_retries == 0)
    {
      if( loraMonitor )
      {
      Serial.println("No reply, is there a listener around?");
      }
      loRaState = 0;
    }
  }
  if( loRaState == 0 )
  {
    tm_retries = 40;
  }
}

// uses global: int8_t ledMode  = 0;    // 0 normal blink, 1 heartbeat blink.
void ledControl() {

  static unsigned long lastTime = 0;
  static unsigned long interval = 100;
  static int state = 0;

  unsigned long tm_now = millis();
  long elapsed = tm_now - lastTime;

// heart beat blinking
  if ( (elapsed) >= interval && (state == 0 || state == 2)) {
    state++;
    lastTime = tm_now;
    digitalWrite(LED_STATUS, HIGH);
    interval = 100;
  }else if ( elapsed >= interval && (state == 1 || state == 3)) {
    state++;
    lastTime = tm_now;
    digitalWrite(LED_STATUS, LOW);
    if( state > 3 && ledMode == 1)
    {
      interval = 500;
    }else{
      interval = 100;
    }
    state &= 0x03;  // ensure 4 states
  }

}
