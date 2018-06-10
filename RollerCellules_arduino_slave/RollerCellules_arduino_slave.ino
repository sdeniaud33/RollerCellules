#define SLAVE_ID 0

// Enable debug prints
// #define MY_DEBUG
// #define MY_DEBUG_VERBOSE
// #define MY_DEBUG_VERBOSE_RF24

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW

//#define MY_RADIO_RFM69
//#define MY_RS485

#include <SPI.h> // Pour la communication via le port SPI
#include <MySensors.h>
#include <nRF24L01.h> // Pour les définitions des registres du nRF24L01

#define MY_NODE_ID 1+SLAVE_ID

// *******************************************************
// **
// ** RADIO MESSAGES
// **
// *******************************************************
const byte RADIO_MSG_RESET[2] = {10, 11};
const byte RADIO_MSG_SET_THRESHOLD[2] = {8, 9};
const byte RADIO_MSG_SLAVE_HEARTBEAT[2] = {2, 3};
const byte RADIO_MSG_SLAVE_CELL_FREE[2] = {4, 5};
const byte RADIO_MSG_SLAVE_CELL_OBSTACLE[2] = {6, 7};

// *******************************************************
// **
// ** PIN DECLARATIONS
// **
// *******************************************************

const int PIN_CELL = A0;
const int PIN_LASER = 3;
const int PIN_OUT_LED_MASTER_HEARTBEAT = 4;
const int PIN_OUT_BUZZER = 6;

// *******************************************************
// **
// ** OTHER DECLARATIONS
// **
// *******************************************************
MyMessage msgHeartbeat(RADIO_MSG_SLAVE_HEARTBEAT[SLAVE_ID], V_DISTANCE);
MyMessage msgCellFree(RADIO_MSG_SLAVE_CELL_FREE[SLAVE_ID], V_STATUS);
MyMessage msgCellObstacle(RADIO_MSG_SLAVE_CELL_OBSTACLE[SLAVE_ID], V_STATUS);
MyMessage msgCellSetThreshold(RADIO_MSG_SET_THRESHOLD[SLAVE_ID], V_DISTANCE);
MyMessage msgCellReset(RADIO_MSG_RESET[SLAVE_ID], V_STATUS);

const int HEARTBEAT_PERIOD = 500 + (SLAVE_ID * 5);
unsigned long lastHeartBeatTime = 0;
int cellValueThreshold = 50;
int minCellValue;
int maxCellValue;				  // LDR value when hightlighted by the laser
bool evenMasterHeartbeat = false; // even / odd flag toggled each time a heartbeat is received from the master module
unsigned long masterLastHeartBeatTime = 0;
unsigned long hbId = 0;

enum cellStatusEnum
{
	cellStatusUnknown,
	cellStatusFree,
	cellStatusObstacle
};

cellStatusEnum lastCellStatus = cellStatusUnknown;

// *******************************************************
// **
// ** PRESENTATION
// **
// *******************************************************
void presentation()
{
	// Send the sketch version information to the gateway
	sendSketchInfo("Slave", "1.0");

	// present(RADIO_MSG_SLAVE_HEARTBEAT[SLAVE_ID], S_BINARY);
	// present(RADIO_MSG_SLAVE_CELL_FREE[SLAVE_ID], S_BINARY);
	// present(RADIO_MSG_SLAVE_CELL_OBSTACLE[SLAVE_ID], S_BINARY);
	// present(RADIO_MSG_SET_THRESHOLD[SLAVE_ID], S_DISTANCE);
	// present(RADIO_MSG_RESET[SLAVE_ID], S_DISTANCE);
}

// *******************************************************
// **
// ** SETUP
// **
// *******************************************************
void setup()
{
	Serial.print("SlaveId = ");
	Serial.println(SLAVE_ID);
	pinMode(PIN_OUT_BUZZER, OUTPUT);

	// ----- Initialization of the cells
	pinMode(PIN_OUT_LED_MASTER_HEARTBEAT, OUTPUT);
	pinMode(PIN_LASER, OUTPUT);
	pinMode(PIN_CELL, INPUT);
	Serial.println("Starting...");
	// Make sure the led is off
	digitalWrite(PIN_LASER, LOW);
	delay(100);
	minCellValue = readCellValue();
	digitalWrite(PIN_LASER, HIGH);
	delay(100);
	maxCellValue = readCellValue();
	Serial.print(minCellValue);
	Serial.print(" <= value <= ");
	Serial.println(maxCellValue);
}

// *******************************************************
// **
// ** RADIO
// **
// *******************************************************

void ackMessageFromServer() {
	for(int i = 0; i < 5; i++) {
		digitalWrite(PIN_LASER, LOW);
		delay(200);
		digitalWrite(PIN_LASER, HIGH);
		delay(50);
	}
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

/*
   Read incoming data from the radio module (receive data from the slave module)
*/
void receive(const MyMessage &message)
{
	Serial.println("RECEIVE");
	if (message.sensor == RADIO_MSG_RESET)
	{
		ackMessageFromServer();
		setup();
		// Ack to server
		send(msgCellReset.set(1));
	}
	else if (message.sensor == RADIO_MSG_SET_THRESHOLD[SLAVE_ID])
	{
		ackMessageFromServer();
		cellValueThreshold = message.getInt();
		Serial.print(">>>>>");
		Serial.println(cellValueThreshold);
		// Ack to server
		send(msgCellSetThreshold.set(cellValueThreshold));
	}
	else if (message.sensor == RADIO_MSG_RESET[SLAVE_ID])
	{
		ackMessageFromServer();
		Serial.println("RESET");
		// Ack to server
		send(msgCellReset.set(1));
		resetFunc();
	}
	else
	{
		Serial.print("Ignoring radio message ");
		Serial.println(message.sensor);
	}
}

void sendRadioHeartBeatIfNeeded()
{
	unsigned long now = millis();
	if (now - lastHeartBeatTime > HEARTBEAT_PERIOD)
	{
		lastHeartBeatTime = now;
		send(msgHeartbeat.set(++hbId));
		Serial.print("hb sent : ");
		Serial.println(hbId);
	}
}

// *******************************************************
// **
// ** CELLS
// **
// *******************************************************

int readCellValue()
{
	return analogRead(PIN_CELL);
}

void sendCellStatus(cellStatusEnum cellStatus)
{
	switch (cellStatus)
	{
	case cellStatusFree:
		send(msgCellFree.set(1));
		Serial.println("Send : free");
		break;
	case cellStatusObstacle:
		send(msgCellObstacle.set(1));
		Serial.println("Send : obstacle");
		beep(50);
		break;
	}
}

void checkCellStatus()
{
	int val = readCellValue();
	//Serial.println(val);
	cellStatusEnum cellStatus = cellStatusUnknown;
	if (val <= minCellValue + cellValueThreshold) {
		cellStatus = cellStatusObstacle;
	}
	else
		cellStatus = cellStatusFree;
	if (cellStatus == lastCellStatus)
	{
		// nothing changed
		return;
	}

	lastCellStatus = cellStatus;
	sendCellStatus(cellStatus);
}

// *******************************************************
// **
// ** BEEP
// **
// *******************************************************
void beep(unsigned char delayms)
{
	/*
    analogWrite(PIN_OUT_BUZZER, 20);
    delay(delayms);
    analogWrite(PIN_OUT_BUZZER, 0);
    delay(delayms);
  */
}

// *******************************************************
// **
// ** LOOP
// **
// *******************************************************
void loop()
{
	sendRadioHeartBeatIfNeeded();
	checkCellStatus();
}
