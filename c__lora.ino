/**
 * Functions for managing the RFM9x LoRa radio
 * 
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
 * 
 */

  /* for Adafruit feather m0  */
#include <SPI.h>
#include <RH_RF95.h>  /* the LoRa RadioHead library from: adafruit link above */

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Set LoRa frequency, must match RX's freq!
#define RF95_FREQ 912.0

// set to 1 to enable waiting for remote LoRa device to send back after this transmits. Only for testing.
#define ENABLE_LORA_RX_ACK  0

int8_t loRaState = 0;   // 0 Tx, 1 Rx, 2 wait ACK

// Singleton instance of the radio driver
RH_RF95 LoRadio(RFM95_CS, RFM95_INT);

void loraSetupPins()
{
 	// setup lora pins
 	// reset pin
 	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);

}

/**
 * reset the radio
 */
void resetLora()
{
	// manual reset LoRa radio
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);	
}

float getLoraFrequency()
{
	return RF95_FREQ;
}
/**
 * setup radio frequency and such
 * returns 0 if OK, 1 if failed init, 2 if failed freq set.
 */
int setupLoraRadio()
{
	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol (7), CRC on

	if(!LoRadio.init())
		return 1;
	if(!LoRadio.setFrequency(RF95_FREQ))
		return 2;

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	LoRadio.setTxPower(23, false);

	return 0;
}

void printLoraRegisters()
{
	// dump registers
	Serial.println("+++ LoRa Radio Registers +++");
	LoRadio.printRegisters(); // only dumps regs: 0 to 0x27
	Serial.println("--- LoRa Radio Registers ---");	
}
// -----------------------------------------------------------------------
// Send a packet of bytes over the LoRa radio
// txPacket is null terminated string, null is not transmitted.
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

//  if ( now - lastTime > interval && loRaState == 0 )
	if ( loRaState == 0 )
	{
		// LoRa state is Tx
		lastTime = now;
		if( get_loraMonitor() )
		{
			Serial.print("LoRa Sending ["); Serial.print(txPacket);Serial.println("]");
			delay(10);
		}
		LoRadio.send((uint8_t *)txPacket, strlen(txPacket));
		if( get_loraMonitor() )
		{
			Serial.println("Waiting for packet to complete...");
			delay(10);
		}
		LoRadio.waitPacketSent();

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
//
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
		if( LoRadio.waitAvailableTimeout(10) )
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


void loraSleep()
{
	// put radio to sleep:
	LoRadio.sleep();

}
// -------------------------------------------------------------------
//  End of file
// -------------------------------------------------------------------
