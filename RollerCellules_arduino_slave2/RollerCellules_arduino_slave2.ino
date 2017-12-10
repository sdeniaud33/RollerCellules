#include <SPI.h>				   // Pour la communication via le port SPI
#include <Mirf.h>				   // Pour la gestion de la communication
#include <nRF24L01.h>			   // Pour les définitions des registres du nRF24L01
#include <MirfHardwareSpiDriver.h> // Pour la communication SPI

#define SLAVE_CELLS_COUNT 1

// *******************************************************
// **
// ** RADIO MESSAGES
// **
// *******************************************************
const byte RADIO_MSG_MASTER_HEARTBEAT = 1;
const byte RADIO_MSG_SLAVE_HEARTBEAT = 2;
const byte RADIO_MSG_SLAVE_CELL_FREE = 3;
const byte RADIO_MSG_SLAVE2_CELL_FREE = 4;
const byte RADIO_MSG_SLAVE_CELL_OBSTACLE = 5;
const byte RADIO_MSG_SLAVE2_CELL_OBSTACLE = 6;
const byte RADIO_MSG_MASTER_MODE_DIAGNOSE = 7;
const byte RADIO_MSG_MASTER_MODE_FREESTART = 8;
const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE = 9;
const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL = 10;
const byte RADIO_MSG_SLAVE2_HEARTBEAT = 11;

// *******************************************************
// **
// ** PIN DECLARATIONS
// **
// *******************************************************

const int PIN_CELL = A0;
const int PIN_LASER = 2;
const int PIN_OUT_LED_MASTER_HEARTBEAT = 4;
const int PIN_OUT_BUZZER = 6;

// *******************************************************
// **
// ** OTHER DECLARATIONS
// **
// *******************************************************
const int HEARTBEAT_PERIOD = 498;
const int CELL_VALUE_DELTA = 30;
unsigned long lastHeartBeatTime = 0;
int defaultCellValue;
bool evenMasterHeartbeat = false; // even / odd flag toggled each time a heartbeat is received from the master module
unsigned long masterLastHeartBeatTime = 0;
enum CellStatusEnum
{
  cellStatus_unknown,
  cellStatus_obstacle,
  cellStatus_free
};

CellStatusEnum lastCellStatus = {cellStatus_unknown};

// *******************************************************
// **
// ** SETUP
// **
// *******************************************************
void setup()
{
  Serial.begin(9600);

  pinMode(PIN_OUT_BUZZER, OUTPUT);

  Mirf.cePin = 9;				 // Broche CE sur D9
  Mirf.csnPin = 10;			 // Broche CSuN sur D10
  Mirf.spi = &MirfHardwareSpi; // On veut utiliser le port SPI hardware
  Mirf.init();				 // Initialise la bibliothèque

  Mirf.channel = 1;			 // Choix du canal de communication (128 canaux disponibles, de 0 à 127)
  Mirf.payload = sizeof(byte); // Taille d'un message (maximum 32 octets)
  Mirf.config();				 // Sauvegarde la configuration dans le module radio

  Mirf.setTADDR((byte *)"nrf02"); // Adresse de transmission
  Mirf.setRADDR((byte *)"nrf01"); // Adresse de réception

  // ----- Initialization of the cells
  pinMode(PIN_OUT_LED_MASTER_HEARTBEAT, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_CELL, INPUT);
  Serial.println("Starting...");
  // Make sure the led is off
  Serial.print("Default value = ");
  digitalWrite(PIN_LASER, LOW);
  defaultCellValue = readCellValue();
  Serial.println(defaultCellValue);
  delay(100);
  // Only switch on the first laser
  digitalWrite(PIN_LASER, HIGH);
}

// *******************************************************
// **
// ** RADIO
// **
// *******************************************************
void sendRadioMessageToMaster(byte value)
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
  if (now - lastHeartBeatTime > HEARTBEAT_PERIOD)
  {
    sendRadioMessageToMaster(RADIO_MSG_SLAVE2_HEARTBEAT);
    lastHeartBeatTime = now;
    Serial.println("hb sent");
  }
}

void readFromRadio()
{
  byte message[1];
  if (!Mirf.isSending() && Mirf.dataReady())
  {
    Mirf.getData(message);
    switch (message[0])
    {
      case RADIO_MSG_SLAVE_HEARTBEAT: // heartbeat from master
        Serial.println("<hb_slave2>");
        break;
      case RADIO_MSG_MASTER_HEARTBEAT: // heartbeat from master
        Serial.println("<hb_master>");
        evenMasterHeartbeat = !evenMasterHeartbeat;
        digitalWrite(PIN_OUT_LED_MASTER_HEARTBEAT, evenMasterHeartbeat ? HIGH : LOW);
        masterLastHeartBeatTime = millis();
        break;
      case RADIO_MSG_MASTER_MODE_FREESTART:
        break;
      case RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE:
        break;
      case RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL:
        break;
      default:
        Serial.println("Ignored");
        break;
    }
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

void sendCellStatus(CellStatusEnum cellStatus)
{
  switch (cellStatus)
  {
    case cellStatus_free:
      Serial.println("Send : free");
      sendRadioMessageToMaster(RADIO_MSG_SLAVE2_CELL_FREE);
      break;
    case cellStatus_obstacle:
      Serial.println("Send : obstacle");
      sendRadioMessageToMaster(RADIO_MSG_SLAVE2_CELL_OBSTACLE);
      beep(50);
      break;
  }
}

void checkCellStatus()
{
  int val = readCellValue();
  CellStatusEnum cellStatus = cellStatus_unknown;
  if (val > defaultCellValue + CELL_VALUE_DELTA)
    cellStatus = cellStatus_free;
  else
    cellStatus = cellStatus_obstacle;
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
  analogWrite(PIN_OUT_BUZZER, 20);
  delay(delayms);
  analogWrite(PIN_OUT_BUZZER, 0);
  delay(delayms);
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
  readFromRadio();
}
