
/* *****************************************************************************
 *
 *           U-Blox GPS functions.
 *
 *
 *  Notes:
 *  To enable UBX protocol, See DS 11.7 How to change between protocols
 *
 * GPS
 * BN-180 GPS/GNSS chip info: https://www.u-blox.com/en/product/ubx-m8030-series#tab-document-resources
 * General info on Adafruit GPS library: https://core-electronics.com.au/tutorials/how-to-use-gps-with-arduino.html
 * Also check out: Bolder Flight Systems UBLOX library.
 * 
 * And SparkFun Ublox Library: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/blob/master/src/SparkFun_Ublox_Arduino_Library.cpp
 *   in Ardiono library: Sparkfun Ublox Arduino Library
 * And GPS Flora / FeatherWing example: GPS_HardwareSerial_Parsing
 * And another ublox lib to look into: https://githubhelp.com/AlessioMorale/ublox
 * 
 *  Our BN-180 device info:
	$GNTXT,01,01,02,u-blox AG - www.u-blox.com*4E
	$GNTXT,01,01,02,HW UBX-M8030 00080000*60
	$GNTXT,01,01,02,ROM CORE 3.01 (107888)*2B
	$GNTXT,01,01,02,FWVER=SPG 3.01*46
	$GNTXT,01,01,02,PROTVER=18.00*11
	$GNTXT,01,01,02,GPS;GLO;GAL;BDS*77
	$GNTXT,01,01,02,SBAS;IMES;QZSS*49
	$GNTXT,01,01,02,GNSS OTP=GPS;GLO*37

 **************************************************************************** */
// adafruit GPS library
#include <Adafruit_GPS.h>

// declare the GPS object. This object provides control the GPS module.
// Serial1 is on pins: RX (pin 0) and TX (pin 1) of Adafruit M0 LoRa Feather
Adafruit_GPS GPS(&Serial1);
#define GPSserial Serial1

/*
 * Setup the GPS 
 */
bool gpsSetup()
{
	// BN-180 GPS is 9600

	//These lines configure the GPS Module
	if( !GPS.begin(9600) )
	{
		return false;
	}
	return true;
}

// GPS command array
// See DS section 32.2 UBX Frame Structure
uint8_t gpsCommandArray[64];

/** ****************************************************************************
 *  Load the Ublox command array with zeros
 **************************************************************************** */
void gpsUbloxArrayZero(uint8_t* pArray, int length)
{
	for(int i = 0; i<length; i++)
	{
		pArray[i] =0;
	}
}
/** ****************************************************************************
 * Load U-Blox header into array
 * See DS section 32.2 UBX Frame Structure
 * pArray - pointer to command array
 * classV - uBlox class ID for message. See DS Section 32.6 UBX Class IDs
 * id     - message ID. See DS Section 32.7 UBX Message Overview
 * ****************************************************************************/
void gpsUbloxSetHeader(uint8_t* pArray, uint8_t classV, uint8_t id, uint8_t length)
{
	pArray[0] = 0xB5;	// Sync bytes ISO8859.1 for mu
	pArray[1] = 0x62; // ASCII 'b'
	pArray[2] = classV; //0x02;	RCVR manager messages
	pArray[3] = id; //0x41;  RXM-PMREQ
	pArray[4] = length;// LSB of length
	pArray[5] = 0;		// MSB of length
}

/** ****************************************************************************
 *  set a payload value of the specified size at the offset+6 in gpsCommandArray[].
 *  data must point to variable in little endian format.
 *  pArray - pointer to command array
 *  offset - 0 to 62
 *  bytes   - 1 to 62
 **************************************************************************** */
void gpsUbloxSetValue(uint8_t* pArray, int8_t offset, int8_t bytes, void* dataVal)
{
	uint8_t* data = (uint8_t*) dataVal;
	for(int i=0; i< bytes; i++)
	{
		// payload starts at index 6 for the array.
		pArray[6 + offset+i] = data[i];
	}
}

/** ****************************************************************************
 * Calculate and Load U-Blox Checksum
 * Must have valid length in the input packet structure
 *  see DS section 32.2 UBX Frame Structure  and  32.4 UBX Checksum
 *  Checksum calcualted over CLASS [idx 2]  to end of Payload
 * pArray - pointer to command array
 * returns length of array including checksum.
 * ****************************************************************************/
int gpsUbloxChecksum(uint8_t* pArray)
{
	uint8_t length = pArray[4]; // we ignore the MSB of length.
	uint8_t CK_A = 0, CK_B = 0;
	// see DS section 32.2 UBX Frame Structure  and  32.4 UBX Checksum
	// calculate checksum over index [2] through length+4
	int i = 2;  // checksum covers class thru payload.
	for( ; i < length +6; i++ )
	{
		CK_A += pArray[i];
		CK_B += CK_A;
	}
	// load checksum
	pArray[i++] = CK_A;
	pArray[i] = CK_B;
	return i;
}

/** ****************************************************************************
 *  Send a Ubox packet out the GPSserial port
 **************************************************************************** */
void gpsUbloxSendPacket(uint8_t* pArray )
{
	int length = gpsUbloxChecksum(pArray);

	for(int index = 0; index <= length; index++)
	{
#if (ENABLE_GPS_SLEEP == 1)
		GPSserial.write(pArray[index]);
#endif
#if (ENABLE_DEBUG_INFO == 1)
		if( gpsMonitorRaw > 0)
		{
			Serial.print(pArray[index], HEX);
			Serial.print(" ");
		}
#endif
	}
}

/** ****************************************************************************
 *  get Ublox Ack or Nak  for UBX-CFG messages only.
 *  For now this has no timeout and will hang forever waiting for ack/nak sequence.
 **************************************************************************** */
int gpsUbloxGetAckNak()
{
#if (ENABLE_GPS_ACKNAK == 1)
	// wait for ack nak
#if (ENABLE_DEBUG_INFO == 1)
	Serial.print(" AckNak: ");
#endif

#if (ENABLE_GPS_SLEEP == 1)
	// wait for ACK NAK
	int acknak1 = 0;
	int acknak2 = 0;
	// find start of UBLOX packet
	while(acknak1 != 0xb5)
	{
		 acknak1 = GPSserial.read();
	}
	while(!GPSserial.available())
		;
	acknak2 = GPSserial.read();
#if (ENABLE_DEBUG_INFO == 1)
	if( gpsMonitorRaw > 0)
	{
		Serial.print(acknak1, HEX);
		Serial.print(" ");
		Serial.print(acknak2, HEX);
		Serial.println(" ");
	}
#endif
#else
#if (ENABLE_DEBUG_INFO == 1)
	Serial.print(" skip - No gps sleep ");
#endif
#endif
#endif  // #if (ENABLE_GPS_ACKNAK == 1)

	return 1; // for now true.
}

/** ****************************************************************************
 * Configure GPS update rate
 * urSeconds - seconds to update
 * DS Sections:
 * 32.10.14.3 Set Message Rate
 * 32.10.23.1 UBX-CFG-RATE (seems to be internal update rate.
 * ****************************************************************************/
void gpsSetUpdateRate(uint16_t urSeconds)
{
	// build a Ublox packet
	gpsUbloxArrayZero(gpsCommandArray, sizeof(gpsCommandArray));

	// set the class/id of the UBX-CFG-RATE command and length, see DS 32.10.23.1
	gpsUbloxSetHeader(gpsCommandArray, 0x06, 0x08, 0x06);
	// set up some payload values
	uint16_t payloadData = urSeconds*1000;  // set milliseconds time
	gpsUbloxSetValue(gpsCommandArray, 0, sizeof(payloadData), &payloadData);
	// set the ratio
	payloadData = urSeconds;
	gpsUbloxSetValue(gpsCommandArray, 2, sizeof(payloadData), &payloadData);

	// Add checksum and Send the packet
	gpsUbloxSendPacket(gpsCommandArray);

}

/** ****************************************************************************
 * Configure GPS to low power mode
 * mode - one of a few modes
 * Not used - not implemented.
 * ****************************************************************************/
void gpsSetLowPower(uint8_t mode)
{
	return;

	gpsCommandArray[0] = 0xff;
	gpsCommandArray[1] = 0;
	// wake the GPS if asleep
	GPS.sendCommand((const char*)gpsCommandArray);
	delay(500);
	// now send config string
	// DS section 32.10.20 UBX-CFG-PMS
	gpsUbloxSetHeader(gpsCommandArray, 0x02, 0x41, 1);
}

/** ****************************************************************************
 * Configure GPS to power off (inactive)
 * mode - one of a few modes
 *
 * For timing examples see DS section 13.2.4 Examples
 * Info about how device wakes up: 13.2.3.2 Wake up
 *
 * ****************************************************************************/
void gpsSetPowerOff(uint8_t mode)
{
	// DS section 13.4 Power On/Off command
	// DS section 32.18.3 UBX-RXM-PMREQ, for longer command with wakeup source 32.18.3.2
//	gpsUbloxHeader(gpsCommandArray, xx,yy, 16);

	//sends command over serial interface to GPS to put it in PMREQ backup mode
	int index;
	// define the RXM_PMREQ command. Has 2 32bit words in little endian format. First is time, second is flags.
	uint8_t UBLOX_GPSStandby[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
	uint32_t payloadData=0;

	// build a Ublox packet
	gpsUbloxArrayZero(gpsCommandArray, sizeof(gpsCommandArray));

	// set the class/id of the UBX-RXM-PMREQ command and length
	gpsUbloxSetHeader(gpsCommandArray, 0x02, 0x41, 0x08);
	// set up some payload values
	payloadData = 0;  // set infinite time
	gpsUbloxSetValue(gpsCommandArray, 0, sizeof(uint32_t), &payloadData);
	// Set the flags
	payloadData = 0x02;
	gpsUbloxSetValue(gpsCommandArray, 4, sizeof(uint32_t), &payloadData);

	// Add checksum and Send the packet
	gpsUbloxSendPacket(gpsCommandArray);

}

/** ****************************************************************************
 *  Wake the GPS up
 **************************************************************************** */
void gpsWakeup()
{
	GPSserial.println();                                   //send some characters to GPS to wake it up
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
 * Section numbers are for u-blox 8-M8 receiver protocol spec.
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
 *  $<GPS Fix 0:1>,<Lat Deg>,<Long Deg>,<Vector speed>,<Vector angle>,<Altitude>,<Sat Count>,<V Battery>,<packet count>
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
//        Serial.print("GPS Found it! ");
				Serial.print("GPS packet [");Serial.print(gpsPacket.c_str());Serial.println("]");
			}
		}
		if( gpsMonitor && gpsMonitorRaw > 1)
		{
			// dump entire data
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
//			String gps = String(GPS.lastNMEA());
//			String tag = gps.substring(0, gps.indexOf(','));
//			if( tag == message)
//			{
//				if(gpsMonitor && (nowTk - lastTime) > interval){
//				}
//			}// if( tag == message)
		}
	}
#if (ENABLE_MCU_SLEEP == 0)

	if ( nowTk - lastTime > interval )
	{
#endif

		// copy GPS data to LoRa gpsPacket_ if delay time elapsed.
		lastTime = nowTk;
		// temporary working buffer
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
		dtostrf(measuredVbat, 2, 2, buff);
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
#if (ENABLE_MCU_SLEEP == 0)
	}else
	{
		strcpy(gpsPacket_, "");
	}
#endif
	return fix;
}

// -------------------------------------------------------------------
//  End of file
// -------------------------------------------------------------------
