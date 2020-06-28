/**
 * functions for controlling the status LED
 */
#define LED_STATUS	(13)	/* LED on pin 13 */

// define these next for additional LEDs
//#define LED_1		(11)
//#define LED_2		(12)
 
#define LED_ON	HIGH
#define LED_OFF	LOW

#define LED_BLINK_FAST  (70)
#define LED_BLINK_      (600)

int8_t ledMode  = 1;    // 0 normal blink, 1 heartbeat blink, 3 LoRa Tx Blink
void set_LedMode( int8_t mode )
{
	ledMode = mode;
}
uint8_t get_LedMode()
{
	return ledMode;
}

// --------------------------------------------------------------------
void ledSetup()
{
	// initialize digital pin 13 as an output.
	pinMode(LED_STATUS, OUTPUT);
	digitalWrite(LED_STATUS, LED_OFF);
	
	#ifdef LED_1
	pinMode(LED_1, OUTPUT);
	digitalWrite(LED_1, LED_OFF);
	#endif

	#ifdef LED_2
	pinMode(LED_2, OUTPUT);
	digitalWrite(LED_2, LED_OFF);
	#endif
}

// --------------------------------------------------------------------
// led blink control
// Call periodically from main loop.
// This is a cooperative multi-task designed function
//
// uses global: int8_t ledMode  = 0;    // 0 normal toggle blink, 1 heartbeat blink.
//
// returns true if heartbeat cycle completed.
// --------------------------------------------------------------------
bool ledControl()
{
	static unsigned long lastTime = 0;
	static unsigned long interval = LED_BLINK_FAST;
	static int state = 0;
	bool cycleDone = false;
	unsigned long tm_now = millis();
	long elapsed = tm_now - lastTime;

// heart beat blinking
	if ( (elapsed) >= interval && (state == 0 || state == 2)) {
		state++;
		lastTime = tm_now;
		digitalWrite(LED_STATUS, LED_ON);
		interval = LED_BLINK_FAST + (state * 20);
	}else if ( elapsed >= interval && (state == 1 || state == 3)) {
		state++;
		lastTime = tm_now;
		digitalWrite(LED_STATUS, LED_OFF);
		if( state > 3 && ledMode == 1)
		{
			// end of heartbeat 2 blinks, so off for longer time.
			interval = LED_BLINK_;
			cycleDone = true;
		}else{
			interval = LED_BLINK_FAST;
		}
		state &= 0x03;  // ensure 4 states
	}
	return cycleDone;
}


// --------------------------------------------------------------------
// blink out code on LED on pin, then pause 500 mSec
// call again to repeat
// --------------------------------------------------------------------
void ledCodePin(int eCode, int LedPin)
{
	digitalWrite(LedPin, LED_OFF);
	delay(500);
	for(int i = 0; i< eCode; i++)
	{
		digitalWrite(LedPin, LED_ON);
		delay(250);
		digitalWrite(LedPin, LED_OFF);
		delay(250);
	}
	delay(500);
}

// --------------------------------------------------------------------
// blink out code on LED_STATUS, then pause 500 mSec
// call again to repeat
// --------------------------------------------------------------------
void ledCode(int eCode)
{
	ledCodePin(eCode, LED_STATUS);
}

void ledOn()
{
	digitalWrite(LED_STATUS, LED_ON);
}

void ledOff()
{
	digitalWrite(LED_STATUS, LED_OFF);
}

#ifdef LED_1
void led1Code(int eCode)
{
	ledCodePin(eCode, LED_1);
}
void led1On()
{
	digitalWrite(LED_1, LED_ON);
}

void led1Off()
{
	digitalWrite(LED_1, LED_OFF);
}
#endif

#ifdef LED_2
void led2Code(int eCode)
{
	ledCodePin(eCode, LED_2);
}
void led2On()
{
	digitalWrite(LED_2, LED_ON);
}

void led2Off()
{
	digitalWrite(LED_2, LED_OFF);
}
#endif


// -------------------------------------------------------------------
//  End of file
// -------------------------------------------------------------------
