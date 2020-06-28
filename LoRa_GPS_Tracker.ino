/*
 * Lora GPS Tracker
 * Lucas Giebler
 * 2019-12-01
 * This is a GPS tracker that sends data over LoRa radio to another node for output.
 * It sends LoRa data to the project LoRa_GPS_Receiver code.
 * Use: to track our cat.
 *
 * IDE Board setting:
 * 		Adafruit SAMD --> "Adafruit M0 Feather" LoRa board
 * or ESP32 Arduino --> "Heltec Wifi Lora 32" LoRa + OLED board (see board target macros below)
 *
 *
 * Power management on feather: https://learn.adafruit.com/adafruit-feather-m0-adalogger/power-management
 * and: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/power-management
 * _mg_ Never use while !Serial after a RTC sleep, it will hang the code.
 * After a Serial.end() followed by begin() Seems like Serial code wants to see a RTS or DTR toggle before returning True.
 *
 * RTC wake up: https://www.arduino.cc/en/Reference/RTC
 * example: https://www.arduino.cc/en/Tutorial/SleepRTCAlarm
 * Sleep mode example: https://github.com/patrickmoffitt/Adafruit-Feather-M0-Motion-Camera
 * Add Library RTCZero (no need to add URL, just search inlibrary manager)
 * Code for it: https://github.com/arduino-libraries/RTCZero/blob/master/src/RTCZero.cpp
 *
 *
 * Low power logging example:
 * https://github.com/cavemoa/Feather-M0-Adalogger
 *
 *
 * 2020-05-28  Basic low power sleep and wake up is working.
 * The thing that was causing trouble was the while(!Serial) loop after a wakeup that will hang the code until USB connected or RTS toggled.
 * 2020-06-27	add debug enable and menu enable if USB connected during setup.
 */

#define myVERSION "1.05"

// how often to get GPS data and send via LoRa
#define GPS_UPDATE_RATE (5000)


// set to 1 to enable putting the LoRa radio in sleep mode after Tx.
#define ENABLE_LORA_SLEEP   1

// set to 1 to enable putting GPS to sleep
#define ENABLE_GPS_SLEEP    1

// set to 1 to enable putting M0 (or ESP32) to sleep after LoRa Tx, wakes via RTC
#define ENABLE_MCU_SLEEP    1

// set to 1 to enable debug data
#define ENABLE_DEBUG_INFO   1

// set to 1 to dump Lora radio registers at start up.
#define ENABLE_LORA_INFO    0

// define board target hardware macro (uncomment only one)
// Board: "adafruit feather M0" LoRa Board
#define FEATHER_M0_LORA

// Board: Heltech board with OLED
//#define HELTECH_OLED_LORA

// ------------------------------------------
#ifdef FEATHER_M0_LORA
 /* for feather m0  */

// define IO pins

#define VBATPIN A7			// vbatt analog input.


// ---  BEGIN  RTC for wake up ---
#if (ENABLE_MCU_SLEEP == 1)
#include <RTCZero.h>
int AlarmTimeSeconds;

/* Create an rtc object */
RTCZero rtc;  // declare the Real Time Clock object

/* Change these values to set the current initial time */
const byte seconds = 0;
#endif	// #if (ENABLE_MCU_SLEEP == 1)

// ---	END	RTC for wake up ---
#endif	/* for feather m0  */
// ------------------------------------------
#define TRACKER_SLEEP_SECONDS	(12)	// how long to sleep between GPS reads and LoRa Tx.
#define TRACKER_WAKE_SECONDS	(9)		// how long to wake and wait before sleep after a fix.

#include <stdlib.h>

// include for dtostrf()
//#include <stdlib_noniso.h>	// https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/stdlib_noniso.h


// BEGIN -- determine RAM remaining in M0 arduino board ----
// sbrk() should be declared in unistd.h but not on Adafruit M0:
extern "C" char *sbrk(int i);
// If this is already defined, then wrong board is selected:
// void* sbrk(ptrdiff_t)	void *	_EXFUN(sbrk,	(ptrdiff_t __incr));
int getFreeRam () {
	char stack_dummy = 0;
	return &stack_dummy - (char*)sbrk(0);
}
// END -- determine RAM remaining in M0 arduino board ----

// --------------------------------------------------------------------
//        global variables:
// --------------------------------------------------------------------
bool usb_connected = false;	// to control debug and power saving modes based on USB connection.
const int usb_connect_wait = 8;	// seconds to wait for a USB connection.

String gpsPacket;
char gpsPacket_[100] = "";
uint16_t packetnum = 0;  // packet counter, we increment per xmission


// --------------------------------------------------------------------
// the setup function runs once when you press reset or power the board
// --------------------------------------------------------------------
void setup() {

	ledSetup();

	loraSetupPins();
	
	ledCode(2); // blink out info code

	// Serial is our command menu port
	Serial.begin(115200);
	// wait for USB connection
	// give extra time after reset to get connected to port for debug or reflashing.
	int usbcnt = usb_connect_wait;	// wait up to 8 seconds.
	do {
		delay(1000);
		if(Serial)
		{
			usb_connected = true;
			Serial.println("Have USB connection.");
			break;
		}
		usbcnt--;
	} while (!Serial && usbcnt);
	
	if( usb_connected )
	{
		// finish waiting to give user time to open serial monitor tool
		delay(usbcnt*1000);
		usbcnt = 0;
	}

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
	if( !gpsSetup() )
	{
		Serial.println("GPS Object begin() Failed");
		ledCode(5); // blink out info code
	}else
	{
#if (ENABLE_MCU_SLEEP == 0 && ENABLE_DEBUG_INFO == 1)
		Serial.println("GPS Object init OK");
#endif
	}

	// These commands do not seem to work on BN-180 device
	//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Sets output to only RMC and GGA sentences
	//GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Sets the output to 1/second.
	//GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // Sets the output to 5 second intervals
	// wakeUp GPS
	gpsWakeup();
	// set update rate of GPS to 1 second
	gpsSetUpdateRate(1);

	// manual reset LoRa radio
	resetLora();

	int radio = setupLoraRadio();
	if( radio == 1 )
	{
		Serial.println("LoRa radio init failed");
	//  Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
		while (1)
			ledCode(3); // blink out error code
	}
#if (ENABLE_MCU_SLEEP == 0 && ENABLE_DEBUG_INFO == 1)
	Serial.println("LoRa radio init OK!");
#endif
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if( radio == 2 ) {
		Serial.println("setFrequency failed");
		while (1)
			ledCode(4); // blink out error code
	}

#if (ENABLE_MCU_SLEEP == 0 && ENABLE_DEBUG_INFO == 1)
	Serial.print("Set Freq to: "); Serial.println(getLoraFrequency());
#endif


#if (ENABLE_LORA_INFO == 1)
	// dump registers
	printLoraRegisters();
	ledCode(2); // blink out info code
	delay(2000);
#endif

// setup RTC to wake up device periodically
#if (ENABLE_MCU_SLEEP == 1)
	rtc.begin();
	rtc.attachInterrupt(alarmMatch);
#if (0)
	Serial.print("Setup: MCU Sleep Seconds: ");
	Serial.println(10);
	AlarmTimeSeconds = rtc.getSeconds()+10; // Adds 10 seconds to alarm time
	AlarmTimeSeconds = AlarmTimeSeconds % 60; // checks for roll over 60 seconds and corrects

	rtc.setAlarmSeconds(AlarmTime); // Wakes at next alarm time
	rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only
	// goto sleep
	rtc.standbyMode();
#endif
#endif
	Serial.println("Starting loop() " myVERSION);
	delay(500);
}

int loops = 0;
volatile int trackerState = 0;	// 0 waiting for fix, 1 = fix and can sleep, 2 = fix and wake time
volatile uint32_t sleepTime = 0;
// -------------------------------------------------------------------
// the loop function runs over and over again forever
// make loop more modular: https://arduino.stackexchange.com/a/37748
// -------------------------------------------------------------------
void loop() {
	#if (ENABLE_DEBUG_INFO == 1)
//  Serial.println("in loop()");
	#endif
	int fix, slept;
	static unsigned long lastTime = 0;
	const unsigned long interval = 500;
	unsigned long tm_now = millis();
	long elapsed = tm_now - lastTime;

	if( elapsed > 500 )
	{
	
		// do any menu actions
		if(usb_connected)
			menuCommand();

		// update battery voltage reading.
		float measuredvbat = getBatteryVoltage();

		// get GPS data and build packet.
		fix = getGPS( gpsPacket_ );
		if( fix && trackerState == 0 )
		{
			trackerState = 1;
			loops = 0;
		}
	
		if( trackerState == 1 || (fix && (loops % 4) == 0) || (loops % 16) == 0 )
		{
			// transmit any GPS info
			// transmit is throttled depending on mode to preserve battery energy
			txLoRa(gpsPacket_);
			// wait a short time for any command from remote
			rxLoRa();
		}
		// If GPS has fix, then go to sleep for a while before sending next fix.
		if(trackerState == 1 && get_gpsMcuSleep() )
		{
#if (ENABLE_DEBUG_INFO == 1)
			Serial.println("Have GPS fix, going to sleep");
#endif
			// set update rate of GPS to 10 seconds
			gpsSetUpdateRate(TRACKER_SLEEP_SECONDS);
	
			systemSleep( TRACKER_SLEEP_SECONDS );
			slept = 1;
			trackerState = 2;
			loops = 0;
			sleepTime = millis() + (1000 * TRACKER_WAKE_SECONDS);
		}
		if( slept )
		{
			// wakeUp GPS
			gpsWakeup();
			// set update rate of GPS to 1 second
			gpsSetUpdateRate(1);
			slept = 0;
		}

#if (ENABLE_DEBUG_INFO == 1)
		if( trackerState == 2 && loops == 0 )
		{
			Serial.println("loop() - Woke Up");
		}
		else if( fix == 0 && (loops % 8)==0){
			printLoopCount(loops);
			Serial.print( "- No Fix, Not sleeping: " myVERSION);
			if(usb_connected)
				Serial.println("  USB:yes");
			else
				Serial.println("  USB:no");
		}
#endif
		loops++;

		// leave GPS on for a while before sleeping again.
		if( fix && trackerState == 2 && millis() > sleepTime )
		{
			trackerState = 1;	// time to sleep.
			loops = 0;
		}else if( !fix )
		{
			// no fix, reset the timer.
			sleepTime = millis() + (1000 * TRACKER_WAKE_SECONDS);
		}

		lastTime = millis();

	}// end of if 500 mSec elapsed time.
	
	ledControl();
}

void printLoopCount(int count)
{
	Serial.print("loop("); Serial.print(count); Serial.print(") ");
}
/** ***********************************************
 *  RTC wake up interrupt handler
 *********************************************** */
void alarmMatch()
{
	ledOn();
//  delay(50);  // allow time for devices to wake up.
	// wakeUp GPS
//  gpsWakeup();
}

/** ****************************************************************************
 * put System to sleep for requested number of seconds
 * Puts LoRa radio, GPS radio, and MCU to sleep
 * ****************************************************************************/
void systemSleep(uint16_t sleepSeconds)
{
	// put GPS to sleep
	// See DS section 32.10.22 UBX-CFG-PWR
	// See DS section 32.10.23 UBX-CFG-RATE
	gpsSetPowerOff(0);

#if (ENABLE_LORA_SLEEP == 1 )
	// put radio to sleep:
	loraSleep();
#endif

	// finally mcu sleep
	mcuSleep( sleepSeconds );
}

/** ****************************************************************************
 * Sleep the MCU for requested number of seconds.
 * Uses the RTC to wake from sleep.
 * ****************************************************************************/
void mcuSleep(uint16_t sleepSeconds)
{
	// LED off
	ledOff();
	// seconds range 1 to 59.

	if( sleepSeconds == 0 )
		sleepSeconds = 1;
	if( sleepSeconds > 58 )
		sleepSeconds = 58;

#if (ENABLE_MCU_SLEEP == 1)
	if(!usb_connected)
	{

#if (ENABLE_DEBUG_INFO == 1)
		Serial.print("mcuSleep() Seconds: ");
		Serial.println(sleepSeconds);
#endif
		Serial.end();

		AlarmTimeSeconds = rtc.getSeconds()+sleepSeconds; // Adds 10 seconds to alarm time
		AlarmTimeSeconds = AlarmTimeSeconds % 60; // checks for roll over 60 seconds and corrects

		rtc.setAlarmSeconds(AlarmTimeSeconds); // Wakes at next alarm time
		rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only
		USBDevice.detach();   // Safely detach the USB prior to sleeping

		rtc.standbyMode();

/*		USBDevice.attach();   // Re-attach the USB ony if it was connected at start up

		if(usb_connected)
		{
			// enable serial port again
			delay(280);
			Serial.begin(115200);
		}
		*/
	}else
#endif  // #if (ENABLE_MCU_SLEEP == 1)
	{
		// no RTC sleep mode, simulate sleep delay. But 4 times shorter.
	#if (ENABLE_DEBUG_INFO == 1)
		Serial.print("mcuSleep() Fake:  Seconds: ");
		Serial.println(sleepSeconds);
	#endif
		// fake sleep
		delay(sleepSeconds * 1000);

	#if (ENABLE_DEBUG_INFO == 1)
		Serial.println("+++ Woke Up +++");
	#endif
	}

	ledOn();
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

float measuredVbat = 0.0;
/** ****************************************************************************
 * getBatteryVoltage reads the battery voltage via ADC
 * returns floating point voltage value
 ******************************************************************************/
float getBatteryVoltage(void)
{
	measuredVbat = analogRead(VBATPIN);
	measuredVbat *= 2;    // Battery Voltage is divided by 2 before input to ADC, so multiply back
	measuredVbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredVbat /= 1024; // convert to voltage by dividing by ADC range (10 bit ADC)
	return measuredVbat;
}


// -------------------------------------------------------------------
//  End of file
// -------------------------------------------------------------------
