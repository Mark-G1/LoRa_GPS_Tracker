/**
 * Menu system
 */
 
// Menu contorl settings
bool gpsMonitor = 0;	// 0 = disabled, 1 = enabled. Send GPS data out serial port
int gpsMonitorRaw = 0;	// 0 = use GPS object. 1 = direct read from Serial1, 2 = all raw data from Serial1.
bool loraMonitor = 0;	//
bool gpsMcuSleep = 1;	// 0 = disable sleep. 1 = enable GPS and MCU sleep even

bool get_gpsMonitor()
{
	return gpsMonitor;
}

bool get_gpsMcuSleep()
{
	return gpsMcuSleep;
}

void set_loraMonitor(bool st)
{
	loraMonitor = st;
}
bool get_loraMonitor()
{
	return loraMonitor;
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

// define the comand menu
const char commandMenu[] = {
	"\n####################################################################\n"
	"#  GPS Tracker LoRa Transmitter " myVERSION "                               #\n"
//  "#  Author: Lucas Giebler                                           #\n"
	"#  Command menu:                                                   #\n"
	"#    0  - show this menu                                           #\n"
	"#    1  - enable/disable monitor of GPS data to serial port        #\n"
	"#    2  - enable/disable monitor of LoRa data to serial port       #\n"
	"#    3  - toggle LED blink modes                                   #\n"
	"#    4  - cycle Raw GPS details (Off|Low|High)                     #\n"
	"#    5  - enable/disable GPS, LoRa and MCU sleep                   #\n"
	"#    9  - show this menu and disable all monitors                  #\n"
	"#                                                                  #\n"
	"####################################################################"
	};
	// manually coordinate values with menu above.
#define MENU_USAGE  0
#define MENU_RESET  9
#define MENU_TOGGLE_MONITOR_GPS		1
#define MENU_TOGGLE_MONITOR_LORA	2
#define MENU_TOGGLE_LED_MODES		3
#define MENU_TOGGLE_GPS_RAW			4
#define MENU_TOGGLE_GPS_SLEEP		5
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
		int pauseTime = 800;  // milliseconds to pause after command
		sscanf(cmd.c_str(), "%d", &cmdNmbr);
		switch(cmdNmbr)
		{
			case MENU_RESET:  // reset monitors and show menu
				gpsMonitor = 0;
				loraMonitor = 0;
				gpsMonitorRaw = 0;
				// no break fall through
			case MENU_USAGE:   // show main menu
				cmdState = menu;
				pauseTime = 1;
				break;
			case MENU_TOGGLE_MONITOR_GPS:
				gpsMonitor ^= 1;  // toggle flag
				Serial.print(" - GPS Monitor: ");Serial.println(get_gpsMonitor()?"ON":"OFF");
				break;
			case MENU_TOGGLE_MONITOR_LORA:
				loraMonitor ^= 1;
				Serial.print(" - LoRa Monitor: ");Serial.println(get_loraMonitor()?"ON":"OFF");
				break;
			case MENU_TOGGLE_LED_MODES:           // LED blink modes
				set_LedMode( get_LedMode() ^ 1 );
				Serial.print(" - LED Mode: ");Serial.println(get_LedMode()?"HeartBeat":"Normal");
				break;
			case MENU_TOGGLE_GPS_RAW:
				gpsMonitorRaw++;
				gpsMonitorRaw &= 0x03;  // cycle through 4 modes.
				if(gpsMonitorRaw > 0 )  gpsMonitor = 1; // turn on general GPS montor mode if needed.
				if(gpsMonitorRaw == 0 ) gpsMonitor = 0;
				Serial.print(" - GPS Raw Monitor: ");
				Serial.print(gpsMonitorRaw?"ON ":"OFF");
				Serial.println(gpsMonitorRaw==1?" Low":gpsMonitorRaw>1?" High":"");
				break;
			case MENU_TOGGLE_GPS_SLEEP:
				gpsMcuSleep ^= 1;
				Serial.print(" - System Sleep: ");Serial.println(gpsMcuSleep?"ON":"OFF");
				break;
			default:
			case -9999:
				Serial.println(" - invalid command");
				cmdState = menu;
				delay(1800);		// delay longer to allow programmer to take control.
				break;
		}
		delay(pauseTime);  // allow user to read any message
	}
}

// -------------------------------------------------------------------
//  End of file
// -------------------------------------------------------------------
