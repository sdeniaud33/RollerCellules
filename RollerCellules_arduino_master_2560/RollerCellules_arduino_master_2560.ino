#include <SPI.h>				   // Pour la communication via le port SPI
#include <Mirf.h>				   // Pour la gestion de la communication
#include <nRF24L01.h>			   // Pour les définitions des registres du nRF24L01
#include <MirfHardwareSpiDriver.h> // Pour la communication SPI
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include "SimpleTimer.h"

#define serialBluetooth Serial1
#define serialMp3 Serial2

#define SLAVE_CELLS_COUNT 1

// *******************************************************
// **
// ** PIN DECLARATIONS
// **
// *******************************************************
#define PIN_OUT_BUZZER 7
#define PIN_OUT_LASER 8 // laser light

#define PIN_RADIO_CE 9   // radio module : CE pin
#define PIN_RADIO_CSN 10 // radio module : CSN pin

#define PIN_OUT_LED_SLAVE_HEARTBEAT 11				// led that will blink each time a heartbeat is received from the salve module
const byte PIN_OUT_SLAVE_CELL_STATUS[2] = {12, 14}; // leds that reflects the slave cell status (on = free, off = obstacle)
#define PIN_OUT_MASTER_CELL_STATUS 13				// led that reflects the master cell status (on = free, off = obstacle)
#define PIN_BTN_MODE 25								// button to switch modes
#define PIN_BTN_RESET 26							// hardware only : not used directly in the code
#define PIN_RADIO_MISO 50							// hardware only : not used directly in the code
#define PIN_RADIO_MOSI 51							// hardware only : not used directly in the code
#define PIN_RADIO_SCK 52							// hardware only : not used directly in the code

#define PIN_IN_ANALOG_LDR A0 // Analog read to get the value of the LDR

// *******************************************************
// **
// ** RADIO MESSAGES
// **
// *******************************************************
const byte RADIO_MSG_MASTER_HEARTBEAT = 1;
const byte RADIO_MSG_SLAVE_HEARTBEAT = 2;
const byte RADIO_MSG_SLAVE_CELL_FREE[2] = {3, 4};
const byte RADIO_MSG_SLAVE_CELL_OBSTACLE[2] = {5, 6};
const byte RADIO_MSG_MASTER_MODE_DIAGNOSE = 7;
const byte RADIO_MSG_MASTER_MODE_FREESTART = 8;
const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE = 9;
const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL = 10;

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

// *******************************************************
// **
// ** OTHER DECLARATIONS
// **
// *******************************************************

const int HEARTBEAT_PERIOD = 500;
const int MP3_ID_ON_YOUR_MARKS_SET = 3;
const int MP3_ID_GO = 2;
const int MP3_ID_SYSTEM_READY = 1;
DFRobotDFPlayerMini myDFPlayer;

bool evenSlaveHeartbeat = false; // even / odd flag toggled each time a heartbeat is received from the slave module
SimpleTimer koSystemStartTimer;
byte slaveCellsCountInUse = 1;

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
	statusUnknown,
	statusFree,
	statusObstacle
};

enum slaveStatusEnum
{
	statusOnline,
	statusOffline
};

bool isRunning[2] = {false, false};
bool isStartingKoSsytem = false;
cellStatusEnum lastMasterCellStatus = statusUnknown;
cellStatusEnum masterCellStatus = statusUnknown;
cellStatusEnum slaveCellStatus[2] = {statusUnknown, statusUnknown};
slaveStatusEnum slaveStatus = statusOffline;

unsigned long masterLastHeartBeatTime = 0;
unsigned long lastSlaveHeartbeatMillis = 0;
long slaveHeartbeatTimeToLive = 1200;
long runningDuration[SLAVE_CELLS_COUNT];
unsigned long runningStartTime;
const int CELL_VALUE_DELTA = 30;
int defaultCellValue;

#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3
LiquidCrystal_I2C lcd(I2C_ADDR, 2, 1, 0, 4, 5, 6, 7);

OneButton btnMode(PIN_BTN_MODE, true);

// *******************************************************
// **
// ** SETUP
// **
// *******************************************************
void setup()
{
	Serial.begin(9600);
	Serial.println("Initializing ...");

	pinMode(PIN_OUT_BUZZER, OUTPUT); // This will set the pin 11 as output

	// ----- Initialization of LCD
	lcd.begin(16, 2);
	// Switch on the backlight
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
	lcd.setBacklight(HIGH);
	lcd.clear();

	// ----- Initialization of the radio module
	Mirf.cePin = PIN_RADIO_CE;
	Mirf.csnPin = PIN_RADIO_CSN;
	Mirf.spi = &MirfHardwareSpi;
	Mirf.init();

	Mirf.channel = 1;
	Mirf.payload = sizeof(byte);
	Mirf.config();

	Mirf.setTADDR((byte *)"nrf01");
	Mirf.setRADDR((byte *)"nrf02");

	// ----- Initialization of the bluetooth module
	serialBluetooth.begin(9600);

	// ----- Initialization of the cells
	pinMode(PIN_OUT_LED_SLAVE_HEARTBEAT, OUTPUT);
	for (int cellId = 0; cellId < SLAVE_CELLS_COUNT; cellId++)
	{
		pinMode(PIN_OUT_SLAVE_CELL_STATUS[cellId], OUTPUT);
	}
	pinMode(PIN_OUT_MASTER_CELL_STATUS, OUTPUT);
	pinMode(PIN_IN_ANALOG_LDR, INPUT);
	pinMode(PIN_OUT_LASER, OUTPUT);
	// Make sure the led is off
	digitalWrite(PIN_OUT_LASER, LOW);
	defaultCellValue = readCellValue();

	// Switch on the led
	digitalWrite(PIN_OUT_LASER, HIGH);

	// ----- Initialization of the mp3 player
	currentMode = modeWaitingForMp3;
	initNewModeDisplay();

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
	myDFPlayer.play(MP3_ID_SYSTEM_READY);

	// ----- Initialization other stuff
	btnMode.attachClick(onClickBtnMode);

	currentMode = modeDiagnose;
	initNewModeDisplay();

	Serial.println("Ready");

	Serial.print("Default value = ");
	Serial.println(defaultCellValue);
}

// *******************************************************
// **
// ** RADIO
// **
// *******************************************************
void sendRadioMessageToSlave(byte value)
{
	byte message[1];
	message[0] = value;
	Mirf.send(message);
	while (Mirf.isSending())
		;
}

void sendRadioHeartBeatIfNeeded()
{
	unsigned long now = millis();
	if (now - masterLastHeartBeatTime > HEARTBEAT_PERIOD)
	{
		sendRadioMessageToSlave(RADIO_MSG_MASTER_HEARTBEAT);
		masterLastHeartBeatTime = now;
		Serial.println("hb sent");
	}
}

/*
   Read incoming data from the radio module (receive data from the slave module)
*/
void readFromRadio()
{
	byte message[1];
	if (!Mirf.isSending() && Mirf.dataReady())
	{
		Mirf.getData(message);
		byte radioMsg = message[0];
		if (radioMsg == RADIO_MSG_SLAVE_HEARTBEAT)
		{
			// heartbeat from slave
			Serial.println("<hb_slave>");
			evenSlaveHeartbeat = !evenSlaveHeartbeat;
			digitalWrite(PIN_OUT_LED_SLAVE_HEARTBEAT, evenSlaveHeartbeat ? HIGH : LOW);
			lastSlaveHeartbeatMillis = millis();
			if (currentMode == modeWaitingForSlave)
			{
				// End of init : start the calibration
				currentMode = modeDiagnose;
				initNewModeDisplay();
			}
		}
		else if (radioMsg >= RADIO_MSG_SLAVE_CELL_FREE[0] && radioMsg <= RADIO_MSG_SLAVE_CELL_FREE[SLAVE_CELLS_COUNT])
		{
			// slave cell status = free
			byte cellId = radioMsg - RADIO_MSG_SLAVE_CELL_FREE[0];
			slaveCellStatus[cellId] = statusFree;
			digitalWrite(PIN_OUT_SLAVE_CELL_STATUS[cellId], HIGH);
			if (currentMode == modeDiagnose)
				updateCurrentModeDisplay();
			Serial.print("free #");
			Serial.println(cellId);
		}
		else if (radioMsg >= RADIO_MSG_SLAVE_CELL_OBSTACLE[0] && radioMsg <= RADIO_MSG_SLAVE_CELL_OBSTACLE[SLAVE_CELLS_COUNT])
		{
			// slave cell status = obstacle
			byte cellId = radioMsg - RADIO_MSG_SLAVE_CELL_OBSTACLE[0];
			digitalWrite(PIN_OUT_SLAVE_CELL_STATUS[cellId], LOW);
			slaveCellStatus[cellId] = statusObstacle;
			if (isRunning[cellId])
				stopChrono(cellId);
			else if (currentMode == modeDiagnose)
				updateCurrentModeDisplay();
			Serial.print("obstacle #");
			Serial.println(cellId);
		}
	}
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
	for (int cellId = 0; cellId < slaveCellsCountInUse; cellId++)
	{
		isRunning[cellId] = false;
	}
	lcd.clear();
	switch (currentMode)
	{
	case modeWaitingForMp3:
		lcd.print("  Cells - v1.0  ");
		lcd.setCursor(0, 1);
		lcd.print("Waiting for MP3 ");
		break;
	case modeWaitingForSlave:
		lcd.setCursor(0, 1);
		lcd.print("No slave in sync");
		break;
	case modeDiagnose:
		lcd.print(" - DIAGNOSIS -  ");
		sendRadioMessageToSlave(RADIO_MSG_MASTER_MODE_DIAGNOSE);
		break;
	case modeFreeStart:
		lcd.print(" - FREE START - ");
		lcd.setCursor(0, 1);
		lcd.print("Ready ...");
		sendRadioMessageToSlave(RADIO_MSG_MASTER_MODE_FREESTART);
		break;
	case modeKoSystem:
		lcd.print(" - K.O SYSTEM - ");
		lcd.setCursor(0, 1);
		lcd.print("Ready ...");
		sendRadioMessageToSlave(RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE);
		break;
	case modeKoSystemDuel:
		lcd.print("  - K.O DUEL -  ");
		lcd.setCursor(0, 1);
		lcd.print("Ready ...");
		sendRadioMessageToSlave(RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL);
		break;
	}
}

/*
   Returns a char to represent a given cell status
*/
char cellStatusToChar(cellStatusEnum cellStatus)
{
	switch (cellStatus)
	{
	case statusFree:
		return 'X';
	case statusObstacle:
		return '|';
	default:
		return '?';
	}
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

	lcd.setCursor(0, 1);
	if (currentMode == modeDiagnose)
	{
		lcd.print(cellStatusToChar(masterCellStatus));
		lcd.print(" -> ");
		for (int cellId = 0; cellId < SLAVE_CELLS_COUNT; cellId++)
		{
			if (cellId > 0)
				lcd.print(" / ");
			lcd.print(cellStatusToChar(slaveCellStatus[cellId]));
		}
	}
	else
	{
		for (int cellId = 0; cellId < slaveCellsCountInUse; cellId++)
		{

			if (isRunning[cellId])
			{
				float f = runningDuration[cellId];
				f = f / 1000;
				lcd.print(f, 3);
				lcd.print(" s");
			}
			else
			{
				// Leave the last duration displayed so that the runner can see its time :)
			}
		}
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
		return;
	if (currentMode == modeWaitingForSlave)
		return;

	if (currentMode == modeDiagnose)
		currentMode = modeFreeStart;
	else if (currentMode == modeFreeStart)
		currentMode = modeKoSystem;
	else
	{
		if (SLAVE_CELLS_COUNT > 1)
		{
			// 2 slave cells
			if (currentMode == modeKoSystem)
				currentMode = modeKoSystemDuel;
			else if (currentMode == modeKoSystemDuel)
				currentMode = modeDiagnose;
			else
				return;
		}
		else
		{
			// Only 1 slave cell
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
   Starts the chrono in 'ko system' mode (single or duel)
*/
void startKoSystem()
{
	if (isStartingKoSsytem)
		return;
	for (int cellId = 0; cellId < SLAVE_CELLS_COUNT; cellId++)
	{
		if (isRunning[cellId])
			return;
	}

	isStartingKoSsytem = true;
	Serial.println("Start KO system");
	// Play the "on your marks .. set ..." mp3 file
	myDFPlayer.play(MP3_ID_ON_YOUR_MARKS_SET);
	lcd.setCursor(0, 1);
	lcd.print("On your marks ...                 ");
	koSystemStartTimer.setTimeout(4000, startKoSystemStepSet);
}

void startKoSystemStepSet()
{
	if (!isStartingKoSsytem)
	{
		myDFPlayer.stop();
		return;
	}
	lcd.setCursor(0, 1);
	lcd.print("SET ...                           ");
	koSystemStartTimer.setTimeout(1000, startKoSystemStepGo);
}

void startKoSystemStepGo()
{
	if (!isStartingKoSsytem)
	{
		myDFPlayer.stop();
		return;
	}
	// Play the "go !!!" mp3 file
	myDFPlayer.play(MP3_ID_GO);
	if (currentMode == modeKoSystem)
	{
		slaveCellsCountInUse = 1;
	}
	else if (currentMode == modeKoSystemDuel)
	{
		slaveCellsCountInUse = SLAVE_CELLS_COUNT;
	}
	for (int cellId = 0; cellId < slaveCellsCountInUse; cellId++)
	{
		isRunning[cellId] = true;
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
	slaveCellsCountInUse = 1;
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
	for (int cellId = 0; cellId < slaveCellsCountInUse; cellId++)
	{
		runningDuration[cellId] = 0;
	}
	lcd.setCursor(0, 1);
	lcd.print("                ");
	updateCurrentModeDisplay();
}

/*
   Stops the chrono
*/
void stopChrono(byte cellId)
{
	isRunning[cellId] = false;
	Serial.println("Stop chrono");
	updateCurrentModeDisplay();
	float f = runningDuration[cellId];
	f = f / 1000;
	serialBluetooth.print(f, 3);
}

void tick()
{
	unsigned long now = millis();
	bool atLeastOneRunning = false;
	for (int cellId = 0; cellId < slaveCellsCountInUse; cellId++)
	{
		if (isRunning[cellId])
		{
			runningDuration[cellId] = (now - runningStartTime) * 1.009; // 1 < x < 1.010
			atLeastOneRunning = true;
		}
	}
	if (atLeastOneRunning)
		updateCurrentModeDisplay();
	else if (currentMode == modeDiagnose)
	{
		updateCurrentModeDisplay();
	}
	slaveStatus = (now - lastSlaveHeartbeatMillis) < slaveHeartbeatTimeToLive ? statusOnline : statusOffline;
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
void checkMasterCellStatus()
{
	int val = readCellValue();
	masterCellStatus = statusUnknown;
	if (val > defaultCellValue + CELL_VALUE_DELTA)
		masterCellStatus = statusFree;
	else
		masterCellStatus = statusObstacle;

	if (masterCellStatus == lastMasterCellStatus)
	{
		// no change
		return;
	}

	lastMasterCellStatus = masterCellStatus;

	for (int cellId = 0; cellId < SLAVE_CELLS_COUNT; cellId++)
	{
		if (isRunning[cellId])
		{
			// should not happen .... never knows
			return;
		}
	}
	if (masterCellStatus == statusObstacle)
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
		beep(50);
	}
}

void beep(unsigned char delayms)
{									 // Created a function for beep
	analogWrite(PIN_OUT_BUZZER, 20); // This will set pin 11 to high
	delay(delayms);					 // Giving a delay
	analogWrite(PIN_OUT_BUZZER, 0);  // This will set pin 11 to LOW
	delay(delayms);					 // Giving a delay
}

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
	if (currentMode == modeWaitingForSlave)
		return;
	if (currentMode == modeWaitingForMp3)
		return;

	if (!serialBluetooth.available())
		return;
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
		for (int cellId = 0; cellId < slaveCellsCountInUse; cellId++)
		{
			stopChrono(cellId);
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
	else
	{
		Serial.println("Not parsed");
	}
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
	sendRadioHeartBeatIfNeeded();
	readFromBluetooth();
	readFromRadio();
	tick();

	if (myDFPlayer.available())
	{
		printDetailFromMp3Player(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
	}
	checkMasterCellStatus();
	btnMode.tick();
	koSystemStartTimer.run();
}
