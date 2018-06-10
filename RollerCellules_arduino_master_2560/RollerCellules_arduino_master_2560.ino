#include <SPI.h>	  // Pour la communication via le port SPI
#include <nRF24L01.h> // Pour les définitions des registres du nRF24L01
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include "SimpleTimer.h"

/*
	WIRING DIAGRAM

	ATMEGA

	SDA					LCD / SDA
	SCL					LCD / SCL
	+5V					LCD / +5V
	GND					LCD / GND

	VIN					BLUETOOTH / +5V
	GND					BLUETOOTH / GND
	TX1					BLUETOOTH / TX
	RX1					BLUETOOTH / RX

	TX3					MP3 / RX
	RX3					MP3 / TX

	A0					LDR
	8					LASER

*/

#define USE_LCD true
#define USE_BLUETOOTH true
#define USE_MP3 true
#define USE_BUZZER false

#define serialBluetooth Serial1
#define serialMp3 Serial2

#define SLAVES_MAX_COUNT 2

// #define MY_DEBUG
// #define MY_DEBUG_VERBOSE
// #define MY_DEBUG_VERBOSE_RF24

#define MY_NODE_ID 6
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_GATEWAY_SERIAL
#include <MySensors.h>

// *******************************************************
// **
// ** PIN DECLARATIONS
// **
// *******************************************************
#if USE_BUZZER
#define PIN_OUT_BUZZER 7
#endif
#define PIN_OUT_LASER 8								  // laser light
#define PIN_RADIO_CE 9								  // radio module : CE pin
#define PIN_RADIO_CSN 10							  // radio module : CSN pin
const byte PIN_OUT_LED_SLAVE_HEARTBEAT[2] = {14, 15}; // led that will blink each time a heartbeat is received from the salve module
const byte PIN_OUT_SLAVE_CELL_STATUS[2] = {11, 12};   // leds that reflects the slave cell status (on = free, off = obstacle)
#define PIN_OUT_MASTER_CELL_STATUS 13				  // led that reflects the master cell status (on = free, off = obstacle)
#define PIN_BTN_MODE 25								  // button to switch modes
#define PIN_BTN_RESET 26							  // hardware only : not used directly in the code
#define PIN_RADIO_MISO 50							  // hardware only : not used directly in the code
#define PIN_RADIO_MOSI 51							  // hardware only : not used directly in the code
#define PIN_RADIO_SCK 52							  // hardware only : not used directly in the code
#define PIN_IN_ANALOG_LDR A0						  // Analog read to get the value of the LDR
#define PIN_FOR_RANDOM_SEED A1						  // Only used for random seed
// *******************************************************
// **
// ** RADIO MESSAGES
// **
// *******************************************************
const byte RADIO_MSG_RESET[2] = {10, 11};
const byte RADIO_MSG_SET_THRESHOLD[2] = {8, 9};
static const byte RADIO_MSG_SLAVE_HEARTBEAT[2] = {2, 3};
static const byte RADIO_MSG_SLAVE_CELL_FREE[2] = {4, 5};
static const byte RADIO_MSG_SLAVE_CELL_OBSTACLE[2] = {6, 7};
MyMessage msgCellSetThresholdSlave0(RADIO_MSG_SET_THRESHOLD[0], V_DISTANCE);
MyMessage msgCellSetThresholdSlave1(RADIO_MSG_SET_THRESHOLD[1], V_DISTANCE);
MyMessage msgCellResetSlave0(RADIO_MSG_RESET[0], V_STATUS);
MyMessage msgCellResetSlave1(RADIO_MSG_RESET[1], V_STATUS);

// *******************************************************
// **
// ** BLUETOOTH MESSAGES
// **
// *******************************************************
const String BLUETOOTH_MSG_START = "<start>";
const String BLUETOOTH_MSG_STOP = "<stop>";
const String BLUETOOTH_MSG_CALIBRATE = "<calibrate>";
const String BLUETOOTH_MSG_FREE_START = "<freestart>";
const String BLUETOOTH_MSG_KO_SYSTEM = "<kosystem>";
const String BLUETOOTH_MSG_KO_SYSTEM_DUEL = "<kosystemduel>";
const String BLUETOOTH_MSG_SET_THRESHOLD = "<threshold.";
const String BLUETOOTH_MSG_RESET = "<reset.";

// *******************************************************
// **
// ** OTHER DECLARATIONS
// **
// *******************************************************
const int HEARTBEAT_PERIOD = 549;
const int MP3_ID_ON_YOUR_MARKS_SET = 3;
const int MP3_ID_GO = 2;
const int MP3_ID_SYSTEM_READY = 1;
DFRobotDFPlayerMini myDFPlayer;

bool evenSlaveHeartbeat[SLAVES_MAX_COUNT] = {false, false}; // even / odd flag toggled each time a heartbeat is received from the slave module
SimpleTimer koSystemStartTimer;
byte slavesCountInUse = 1;

// Enum for the status of the module (master module)
enum modeEnum
{
	modeWaitingForMp3,   // the MP3 player is initializing
	modeWaitingForSlave, // waiting for the first heartbeat from then slave module
	modeDiagnose,		 // diagnosis : mainly show cells alignments
	modeFreeStart,		 // 'free start' mode
	modeKoSystem,		 // 'k.o system' mode
	modeKoSystemDuel	 // 'k.o system' mode (duel)
};

modeEnum currentMode;

enum cellStatusEnum
{
	cellStatusUnknown,
	cellStatusFree,
	cellStatusObstacle
};

enum slaveStatusEnum
{
	statusOnline,
	statusOffline
};

bool isRunning[2] = {false, false};
bool isStartingKoSsytem = false;
cellStatusEnum lastCellStatus = cellStatusUnknown;
cellStatusEnum cellStatus = cellStatusUnknown;
cellStatusEnum slaveCellStatus[2] = {cellStatusUnknown, cellStatusUnknown};
slaveStatusEnum slaveStatus[2] = {statusOffline, statusOffline};
unsigned long masterLastHeartBeatTime = 0;
unsigned long lastSlaveHeartbeatMillis[SLAVES_MAX_COUNT] = {0, 0};
long slaveHeartbeatTimeToLive = 1200;
long runningDuration[SLAVES_MAX_COUNT];
unsigned long runningStartTime;
unsigned int cellThreshold = 50;
int maxCellValue; // LDR value when hightlighted by the laser
int minCellValue;

#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3
LiquidCrystal_I2C lcd(I2C_ADDR, 2, 1, 0, 4, 5, 6, 7);

OneButton btnMode(PIN_BTN_MODE, true);

// *******************************************************
// **
// ** PRESENTATION
// **
// *******************************************************
void presentation()
{
	// Present locally attached sensors
}
// *******************************************************
// **
// ** SETUP
// **
// *******************************************************
void setup()
{
	msgCellSetThresholdSlave0.setDestination(1); // 1 = Slave#0
	msgCellSetThresholdSlave1.setDestination(2); // 2 = Slave#1
	msgCellResetSlave0.setDestination(1);		 // 1 = Slave#0
	msgCellResetSlave1.setDestination(2);		 // 2 = Slave#1
	randomSeed(analogRead(PIN_FOR_RANDOM_SEED));
	Serial.println("Initializing ...");

#if USE_BUZZER
	pinMode(PIN_OUT_BUZZER, OUTPUT);
	digitalWrite(PIN_OUT_BUZZER, LOW);
#endif
#if USE_LCD
	// ----- Initialization of LCD
	lcd.begin(16, 2);
	// Switch on the backlight
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
	lcd.setBacklight(HIGH);
	lcd.clear();
#endif

#if USE_BLUETOOTH
	// ----- Initialization of the bluetooth module
	serialBluetooth.begin(9600);
	Serial.println("Bluetooth : OK");
#endif
	// ----- Initialization of the cells
	for (int slaveId = 0; slaveId < SLAVES_MAX_COUNT; slaveId++)
	{
		pinMode(PIN_OUT_LED_SLAVE_HEARTBEAT[slaveId], OUTPUT);
		pinMode(PIN_OUT_SLAVE_CELL_STATUS[slaveId], OUTPUT);
	}
	pinMode(PIN_OUT_MASTER_CELL_STATUS, OUTPUT);
	pinMode(PIN_IN_ANALOG_LDR, INPUT);
	pinMode(PIN_OUT_LASER, OUTPUT);

	// Make sure the led is off
	digitalWrite(PIN_OUT_LASER, LOW);
	delay(100);
	minCellValue = readCellValue();
	digitalWrite(PIN_OUT_LASER, HIGH);
	delay(100);
	maxCellValue = readCellValue();
	Serial.print(minCellValue);
	Serial.print(" <= value <= ");
	Serial.println(maxCellValue);

	// ----- Initialization of the mp3 player
	currentMode = modeWaitingForMp3;
	initNewModeDisplay();
#if USE_MP3
	serialMp3.begin(9600);
	Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
	if (!myDFPlayer.begin(serialMp3))
	{ //Use softwareSerial to communicate with mp3.
		Serial.println(F("Unable to begin:"));
		Serial.println(F("1.Please recheck the connection!"));
		Serial.println(F("2.Please insert the SD card!"));
		while (true)
			;
	}
	Serial.println("mp3.enable DAC");
	myDFPlayer.enableDAC();
	Serial.println("mp3.setVolume");
	myDFPlayer.volume(30); //Set volume value. From 0 to 30
	Serial.println("mp3.play ready");
	myDFPlayer.play(MP3_ID_SYSTEM_READY);
#endif
	// ----- Initialization other stuff
	btnMode.attachClick(onClickBtnMode);

	currentMode = modeDiagnose;
	Serial.println("ready to display");
	initNewModeDisplay();
#if USE_BUZZER
	beep(200);
#endif
	Serial.println("Ready");
}

// *******************************************************
// **
// ** DISPLAY
// **
// *******************************************************
/*
   Inits the display for a new mode (should be invoked everytime currentMode is changed) - 1st line of the LCD
*/
void initNewModeDisplay()
{
	isStartingKoSsytem = false;
	for (int slaveId = 0; slaveId < slavesCountInUse; slaveId++)
	{
		isRunning[slaveId] = false;
	}
	displayCurrentMode();
}

void displayCurrentMode()
{
#if USE_LCD
	lcd.clear();
	switch (currentMode)
	{
	case modeWaitingForMp3:
		lcd.print(" Cellules - 1.0 ");
		lcd.setCursor(0, 1);
		lcd.print("Attente MP3     ");
		break;
	case modeWaitingForSlave:
		lcd.setCursor(0, 1);
		lcd.print("No slave in sync");
		break;
	case modeDiagnose:
		lcd.print("-DIAGNOSTIQUES- ");
		break;
	case modeFreeStart:
		lcd.print("  -FREE START-  ");
		lcd.setCursor(0, 1);
		lcd.print("Ready ...       ");
		break;
	case modeKoSystem:
		lcd.print("  -K.O SYSTEM-  ");
		lcd.setCursor(0, 1);
		lcd.print("Ready ...       ");
		break;
	case modeKoSystemDuel:
		lcd.print("   -K.O DUEL-   ");
		lcd.setCursor(0, 1);
		lcd.print("Ready ...       ");
		break;
	}
#endif
}

void displayTransientMessage(String msg1, String msg2, long duration = 2000)
{
	lcd.clear();
	lcd.print(msg1);
	lcd.setCursor(0, 1);
	lcd.print(msg2);
	delay(duration);
	displayCurrentMode();
}
/*
   Update the display for the current mode (2nd line on LCD)
*/
void updateCurrentModeDisplay()
{
	if (currentMode == modeWaitingForSlave)
		return;
	if (currentMode == modeWaitingForMp3)
		return;
#if USE_LCD
	lcd.setCursor(0, 1);
	if (currentMode == modeDiagnose)
	{
		lcd.print(cellStatusToChar(cellStatus));
		lcd.print(" -> ");
		for (int slaveId = 0; slaveId < SLAVES_MAX_COUNT; slaveId++)
		{
			if (slaveId > 0)
				lcd.print(" / ");
			lcd.print(cellStatusToChar(slaveCellStatus[slaveId]));
		}
	}
	else
	{
		if (isRunning[0] || (slavesCountInUse == 2 && isRunning[1]))
		{
			// At least on runner is running
			for (int slaveId = 0; slaveId < slavesCountInUse; slaveId++)
			{
				if (slaveId > 0)
					lcd.print(" / ");
				float f = runningDuration[slaveId];
				f = f / 1000;
				lcd.print(f, 3);
			}
			if (slavesCountInUse == 1)
				lcd.println(" s         ");
		}
		else
		{
			// Leave the last duration displayed so that the runner(s) can see its time :)
		}
	}
#endif
}

// *******************************************************
// **
// ** RADIO
// **
// *******************************************************

byte getSlaveId(byte radioMsg)
{
	if ((radioMsg == RADIO_MSG_SLAVE_CELL_FREE[1]) ||
		(radioMsg == RADIO_MSG_SLAVE_CELL_OBSTACLE[1]) ||
		(radioMsg == RADIO_MSG_SLAVE_HEARTBEAT[1]))
		return 1;
	return 0;
}

/*
   Read incoming data from the radio module (receive data from the slave module)
*/
void receive(const MyMessage &message)
{
	if ((message.sensor == RADIO_MSG_SLAVE_HEARTBEAT[0]) ||
		(message.sensor == RADIO_MSG_SLAVE_HEARTBEAT[1]))
	{
		// heartbeat from slave
		int slaveId = getSlaveId(message.sensor);
		Serial.print("<hb_slave.");
		Serial.print(slaveId);
		Serial.print(".");
		Serial.print(message.getULong());
		Serial.println(">");
		evenSlaveHeartbeat[slaveId] = !evenSlaveHeartbeat[slaveId];

		digitalWrite(PIN_OUT_LED_SLAVE_HEARTBEAT[slaveId], evenSlaveHeartbeat[slaveId] ? HIGH : LOW);
#if USE_LCD
		lcd.setCursor(14 + slaveId, 0);
		lcd.print(evenSlaveHeartbeat[slaveId] ? "*" : " ");
#endif
		lastSlaveHeartbeatMillis[slaveId] = millis();
		if (currentMode == modeWaitingForSlave)
		{
			// End of init : start the calibration
			currentMode = modeDiagnose;
			initNewModeDisplay();
		}
	}
	else if ((message.sensor == RADIO_MSG_SLAVE_CELL_FREE[0]) ||
			 (message.sensor == RADIO_MSG_SLAVE_CELL_FREE[1]))
	{
		// slave cell status = free
		int slaveId = getSlaveId(message.sensor);
		slaveCellStatus[slaveId] = cellStatusFree;
		digitalWrite(PIN_OUT_SLAVE_CELL_STATUS[slaveId], HIGH);
		if (currentMode == modeDiagnose)
			updateCurrentModeDisplay();
		Serial.print("free #");
		Serial.println(slaveId);
	}
	else if ((message.sensor == RADIO_MSG_SLAVE_CELL_OBSTACLE[0]) ||
			 (message.sensor == RADIO_MSG_SLAVE_CELL_OBSTACLE[1]))
	{
		// slave cell status = obstacle
		int slaveId = getSlaveId(message.sensor);
		digitalWrite(PIN_OUT_SLAVE_CELL_STATUS[slaveId], LOW);
		slaveCellStatus[slaveId] = cellStatusObstacle;
		if (isRunning[slaveId])
			stopChrono(slaveId);
		else if (currentMode == modeDiagnose)
			updateCurrentModeDisplay();
		Serial.print("obstacle #");
		Serial.println(slaveId);
	}
	else if (message.sensor == RADIO_MSG_SET_THRESHOLD[0])
	{
		// Just an aknowledge message from slave #0
		displayTransientMessage("Ack.threshold", "Slave #0 = " + String(message.getInt()));
	}
	else if (message.sensor == RADIO_MSG_SET_THRESHOLD[1])
	{
		// Just an aknowledge message from slave #1
		displayTransientMessage("Ack.threshold", "Slave #1 = " + String(message.getInt()));
	}
	else if (message.sensor == RADIO_MSG_RESET[0])
	{
		// Just an aknowledge message from slave #0
		displayTransientMessage("Ack.reset", "Slave #0");
	}
	else if (message.sensor == RADIO_MSG_RESET[1])
	{
		// Just an aknowledge message from slave #1
		displayTransientMessage("Ack.reset", "Slave #1");
	}
	else
	{
		Serial.print("Ignoring radio message ");
		Serial.println(message.sensor);
	}
}

// *******************************************************
// **
// ** BUTTONS
// **
// *******************************************************

/*
   This callback is invoked everytime the 'mode' button is clicked
*/
void onClickBtnMode()
{
	Serial.println("Click");
	if (currentMode == modeWaitingForMp3)
	{
		Serial.println("Btn : waiting for mp3");
		return;
	}
	if (currentMode == modeWaitingForSlave)
	{
		Serial.println("Btn : waiting for slave");
		return;
	}
	if (currentMode == modeDiagnose)
		currentMode = modeFreeStart;
	else if (currentMode == modeFreeStart)
		currentMode = modeKoSystem;
	else
	{
		if (SLAVES_MAX_COUNT > 1)
		{
			// 2 slaves
			if (currentMode == modeKoSystem)
				currentMode = modeKoSystemDuel;
			else if (currentMode == modeKoSystemDuel)
				currentMode = modeDiagnose;
			else
				return;
		}
		else
		{
			// Only 1 slave
			if (currentMode == modeKoSystem)
				currentMode = modeDiagnose;
			else
				return;
		}
	}
	initNewModeDisplay();
}

// *******************************************************
// **
// ** CHRONOS
// **
// *******************************************************

/*
   Starts the chrono in 'ko system' mode
*/
void startKoSystem()
{
	if (isStartingKoSsytem)
		return;
	for (int slaveId = 0; slaveId < SLAVES_MAX_COUNT; slaveId++)
	{
		if (isRunning[slaveId])
			return;
	}

	isStartingKoSsytem = true;
	Serial.println("Start KO system");
// Play the "on your marks .. set ..." mp3 file
#if USE_MP3
	myDFPlayer.play(MP3_ID_ON_YOUR_MARKS_SET);
#endif
#if USE_LCD
	lcd.setCursor(0, 1);
	lcd.print("On your marks...");
#endif
	koSystemStartTimer.setTimeout(4000, startKoSystemStepSet);
}

void startKoSystemStepSet()
{
	if (!isStartingKoSsytem)
	{
#if USE_MP3
		myDFPlayer.stop();
#endif
		return;
	}
#if USE_LCD
	lcd.setCursor(0, 1);
	lcd.print("SET ...         ");
#endif
	long pauseDuration = 1000 + random(2000);
	Serial.print("Pause = ");
	Serial.println(pauseDuration);
	koSystemStartTimer.setTimeout(pauseDuration, startKoSystemStepGo);
}

void startKoSystemStepGo()
{
	if (!isStartingKoSsytem)
	{
#if USE_MP3
		myDFPlayer.stop();
#endif
		return;
	}
#if USE_MP3
	// Play the "go !!!" mp3 file
	myDFPlayer.play(MP3_ID_GO);
#endif
	if (currentMode == modeKoSystem)
	{
		slavesCountInUse = 1;
	}
	else if (currentMode == modeKoSystemDuel)
	{
		slavesCountInUse = 2;
	}
	for (int slaveId = 0; slaveId < slavesCountInUse; slaveId++)
	{
		isRunning[slaveId] = true;
	}
	startChrono();
}

/*
   Starts the chrono in 'free start' mode
*/
void startFreeStart()
{
	if (isStartingKoSsytem)
		return;
	Serial.println("Start Free start");
	slavesCountInUse = 1;
	isRunning[0] = true;
	startChrono();
}

/*
   Starts the chrono - should not be invoked : use startFreeStart() or startKoSystem() instead
*/
void startChrono()
{
	isStartingKoSsytem = false;
	runningStartTime = millis();
	for (int slaveId = 0; slaveId < slavesCountInUse; slaveId++)
		runningDuration[slaveId] = 0;
#if USE_LCD
	lcd.setCursor(0, 1);
	lcd.print("                ");
#endif
	updateCurrentModeDisplay();
}

/*
   Stops the chrono
*/
void stopChrono(byte slaveId)
{
	isRunning[slaveId] = false;
	Serial.print("Stop chrono");
	Serial.println(slaveId);
	updateCurrentModeDisplay();
#if USE_BLUETOOTH
	float f = runningDuration[slaveId];
	f = f / 1000;
	serialBluetooth.print(":T");
	serialBluetooth.print(slaveId);
	serialBluetooth.print(f, 3);
#endif
}

void tick()
{
	unsigned long now = millis();
	bool atLeastOneRunning = false;
	for (int slaveId = 0; slaveId < slavesCountInUse; slaveId++)
	{
		if (isRunning[slaveId])
		{
			runningDuration[slaveId] = (now - runningStartTime) * 1.009; // 1 < x < 1.010
			atLeastOneRunning = true;
		}
		slaveStatus[slaveId] = (now - lastSlaveHeartbeatMillis[slaveId]) < slaveHeartbeatTimeToLive ? statusOnline : statusOffline;
	}
	if (atLeastOneRunning)
		updateCurrentModeDisplay();
	else if (currentMode == modeDiagnose)
	{
		updateCurrentModeDisplay();
	}
}

// *******************************************************
// **
// ** CELLS
// **
// *******************************************************

int readCellValue()
{
	return analogRead(PIN_IN_ANALOG_LDR);
}

/*
   Check status of the start cell
*/
void checkCellStatus()
{
	int val = readCellValue();
	//Serial.println(val);
	cellStatus = cellStatusUnknown;
	if (val <= minCellValue + cellThreshold)
	{
		cellStatus = cellStatusObstacle;
		// Serial.print("OBSTABCLE ");
		// Serial.println(val);
	}
	else
		cellStatus = cellStatusFree;

	if (cellStatus == lastCellStatus)
	{
		// no change
		return;
	}

	lastCellStatus = cellStatus;
	for (int slaveId = 0; slaveId < SLAVES_MAX_COUNT; slaveId++)
	{
		if (isRunning[slaveId])
		{
			// should not happen .... never knows
			return;
		}
	}
	if (cellStatus == cellStatusObstacle)
	{
		// The beam has been interrupted
		switch (currentMode)
		{
		case modeKoSystem:
		case modeKoSystemDuel:
			startKoSystem();
			break;
		case modeFreeStart:
			startFreeStart();
			break;
		}
#if USE_BUZZER
		beep(50);
#endif
	}
}

/*
   Returns a char to represent a given cell status
*/
char cellStatusToChar(cellStatusEnum cellStatus)
{
	switch (cellStatus)
	{
	case cellStatusFree:
		return 'X';
	case cellStatusObstacle:
		return '|';
	default:
		return '?';
	}
}

// *******************************************************
// **
// ** BEEP
// **
// *******************************************************
#if USE_BUZZER
void beep(unsigned char delayms)
{
	digitalWrite(PIN_OUT_BUZZER, HIGH);
	delay(delayms);
	digitalWrite(PIN_OUT_BUZZER, LOW);
	delay(delayms);
}
#endif

// *******************************************************
// **
// ** BLUETOOTH
// **
// *******************************************************

/*
   Read incoming data from bluetooth
*/
void readFromBluetooth()
{
#if USE_BLUETOOTH
	if (currentMode == modeWaitingForSlave)
		return;
	if (currentMode == modeWaitingForMp3)
		return;
	if (!serialBluetooth.available())
		return;
	Serial.print(".");
	String str = serialBluetooth.readStringUntil('\n');
	Serial.print("- BLUETOOTH -");
	Serial.print(str);
	Serial.println("--");

	if (str[str.length() - 1] == '\r')
	{
		str.remove(str.length() - 1, 1);
	}

	if (str.equals(BLUETOOTH_MSG_START))
	{
		Serial.println("Start");
		switch (currentMode)
		{
		case modeFreeStart:
			startFreeStart();
			break;
		case modeKoSystem:
		case modeKoSystemDuel:
			startKoSystem();
			break;
		}
	}
	else if (str.equals(BLUETOOTH_MSG_STOP))
	{
		Serial.println("Stop");
		for (int slaveId = 0; slaveId < slavesCountInUse; slaveId++)
		{
			stopChrono(slaveId);
		}
	}
	else if (str.equals(BLUETOOTH_MSG_CALIBRATE))
	{
		currentMode = modeDiagnose;
		initNewModeDisplay();
	}
	else if (str.equals(BLUETOOTH_MSG_FREE_START))
	{
		currentMode = modeFreeStart;
		initNewModeDisplay();
	}
	else if (str.equals(BLUETOOTH_MSG_KO_SYSTEM))
	{
		currentMode = modeKoSystem;
		initNewModeDisplay();
	}
	else if (str.equals(BLUETOOTH_MSG_KO_SYSTEM_DUEL))
	{
		currentMode = modeKoSystemDuel;
		initNewModeDisplay();
	}
	else if (str.startsWith(BLUETOOTH_MSG_SET_THRESHOLD))
	{
		String target = str.substring(BLUETOOTH_MSG_SET_THRESHOLD.length(), BLUETOOTH_MSG_SET_THRESHOLD.length() + 1);
		String val = str.substring(BLUETOOTH_MSG_SET_THRESHOLD.length() + 2, str.length() - 1); // , 1);
		if (target.equals("m"))
		{
			Serial.print("Threshold master = [");
			Serial.print(val);
			Serial.println("]");
			cellThreshold = val.toInt();
			displayTransientMessage("Threshold", "master = " + val);
		}
		else if (target.equals("0"))
		{
			int threshold = val.toInt();
			Serial.print("Threshold slave #0 = [");
			Serial.print(threshold);
			Serial.println("]");
			send(msgCellSetThresholdSlave0.set(threshold));
		}
		else if (target.equals("1"))
		{
			int threshold = val.toInt();
			Serial.print("Threshold slave #1 = [");
			Serial.print(threshold);
			Serial.println("]");
			send(msgCellSetThresholdSlave1.set(threshold));
		}
	}
	else if (str.startsWith(BLUETOOTH_MSG_RESET))
	{
		String target = str.substring(BLUETOOTH_MSG_RESET.length(), BLUETOOTH_MSG_RESET.length() + 1);
		if (target.equals("0"))
		{
			Serial.println("Reset slave #0");
			send(msgCellResetSlave0.set(1));
		}
		else if (target.equals("1"))
		{
			Serial.println("Reset slave #1");
			send(msgCellResetSlave1.set(1));
		}
	}
	else
	{
		Serial.println("Not parsed");
	}
#endif
}

// *******************************************************
// **
// ** MP3
// **
// *******************************************************

void printDetailFromMp3Player(uint8_t type, int value)
{
	Serial.print("Output from MP3 player : ");
	switch (type)
	{
	case TimeOut:
		Serial.println(F("Time Out!"));
		break;
	case WrongStack:
		Serial.println(F("Stack Wrong!"));
		break;
	case DFPlayerCardInserted:
		Serial.println(F("Card Inserted!"));
		break;
	case DFPlayerCardRemoved:
		Serial.println(F("Card Removed!"));
		break;
	case DFPlayerCardOnline:
		Serial.println(F("Card Online!"));
		break;
	case DFPlayerPlayFinished:
		Serial.print(F("Number:"));
		Serial.print(value);
		Serial.println(F(" Play Finished!"));
		break;
	case DFPlayerError:
		Serial.print(F("DFPlayerError:"));
		switch (value)
		{
		case Busy:
			Serial.println(F("Card not found"));
			break;
		case Sleeping:
			Serial.println(F("Sleeping"));
			break;
		case SerialWrongStack:
			Serial.println(F("Get Wrong Stack"));
			break;
		case CheckSumNotMatch:
			Serial.println(F("Check Sum Not Match"));
			break;
		case FileIndexOut:
			Serial.println(F("File Index Out of Bound"));
			break;
		case FileMismatch:
			Serial.println(F("Cannot Find File"));
			break;
		case Advertise:
			Serial.println(F("In Advertise"));
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

// *******************************************************
// **
// ** LOOP
// **
// *******************************************************

void loop()
{
	readFromBluetooth();
	tick();
#if USE_MP3
	if (myDFPlayer.available())
	{
		printDetailFromMp3Player(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
	}
#endif
	checkCellStatus();
	btnMode.tick();
	koSystemStartTimer.run();
}
