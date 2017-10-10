#include <SPI.h>      // Pour la communication via le port SPI
#include <Mirf.h>     // Pour la gestion de la communication
#include <nRF24L01.h> // Pour les définitions des registres du nRF24L01
#include <MirfHardwareSpiDriver.h> // Pour la communication SPI

#define SLAVE_CELLS_COUNT 2

// *******************************************************
// **
// ** RADIO MESSAGES
// **
// *******************************************************
const byte RADIO_MSG_MASTER_HEARTBEAT = 1;
const byte RADIO_MSG_SLAVE_HEARTBEAT = 2;
const byte RADIO_MSG_SLAVE_CELL_FREE[SLAVE_CELLS_COUNT] = {3, 4};
const byte RADIO_MSG_SLAVE_CELL_OBSTACLE[SLAVE_CELLS_COUNT] = {5, 6};
const byte RADIO_MSG_MASTER_MODE_DIAGNOSE = 7;
const byte RADIO_MSG_MASTER_MODE_FREESTART = 8;
const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE = 9;
const byte RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL = 10;

// *******************************************************
// **
// ** PIN DECLARATIONS
// **
// *******************************************************

const int PIN_CELL[SLAVE_CELLS_COUNT] = {A0, A1};
const int PIN_LASER[SLAVE_CELLS_COUNT] = {2, 3};
const int PIN_OUT_LED_MASTER_HEARTBEAT = 4;

// *******************************************************
// **
// ** OTHER DECLARATIONS
// **
// *******************************************************
const int HEARTBEAT_PERIOD = 500;
const int CELL_VALUE_DELTA = 30;
unsigned long lastHeartBeatTime = 0;
int defaultCellValue[2];
bool evenMasterHeartbeat = false;       // even / odd flag toggled each time a heartbeat is received from the master module
unsigned long masterLastHeartBeatTime = 0;
byte cellsCountInUse = 1; // The number of cells actually in use [1..SLAVE_CELLS_COUNT]

enum CellStatusEnum {
  cellStatus_unknown,
  cellStatus_obstacle,
  cellStatus_free
};

CellStatusEnum lastCellStatus[SLAVE_CELLS_COUNT] = {cellStatus_unknown, cellStatus_unknown};

// *******************************************************
// **
// ** SETUP
// **
// *******************************************************
void setup() {
  Serial.begin(9600);

  Mirf.cePin = 9; // Broche CE sur D9
  Mirf.csnPin = 10; // Broche CSuN sur D10
  Mirf.spi = &MirfHardwareSpi; // On veut utiliser le port SPI hardware
  Mirf.init(); // Initialise la bibliothèque

  Mirf.channel = 1; // Choix du canal de communication (128 canaux disponibles, de 0 à 127)
  Mirf.payload = sizeof(byte); // Taille d'un message (maximum 32 octets)
  Mirf.config(); // Sauvegarde la configuration dans le module radio

  Mirf.setTADDR((byte *) "nrf02"); // Adresse de transmission
  Mirf.setRADDR((byte *) "nrf01"); // Adresse de réception

  // ----- Initialization of the cells
  pinMode(PIN_OUT_LED_MASTER_HEARTBEAT, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_CELL[0], INPUT);
  pinMode(PIN_CELL[1], INPUT);
  Serial.println("Starting...");
  // Make sure the led is off
  Serial.print("Default value = ");
  for (int cellId = 0; cellId < SLAVE_CELLS_COUNT; cellId++) {
    digitalWrite(PIN_LASER[cellId], LOW);
    defaultCellValue[cellId] = readCellValue(cellId);
    Serial.print(defaultCellValue[cellId]);
    Serial.print(" / ");
  }
  Serial.println();
  delay(100);
  // Only switch on the first laser
  digitalWrite(PIN_LASER[0], HIGH);
}

// *******************************************************
// **
// ** RADIO
// **
// *******************************************************
void sendRadioMessageToMaster(byte value) {
  byte message[1];
  message[0] = value;
  Mirf.send(message);
  while (Mirf.isSending());
}

void sendRadioHeartBeatIfNeeded() {
  unsigned long now = millis();
  if (now - lastHeartBeatTime > HEARTBEAT_PERIOD) {
    sendRadioMessageToMaster(RADIO_MSG_SLAVE_HEARTBEAT);
    lastHeartBeatTime = now;
    Serial.println("hb sent");
  }
}

void readFromRadio() {
  byte message[1];
  if (!Mirf.isSending() && Mirf.dataReady()) {
    Mirf.getData(message);
    switch (message[0]) {
      case RADIO_MSG_MASTER_HEARTBEAT : // heartbeat from master
        Serial.println("<hb_master>");
        evenMasterHeartbeat = !evenMasterHeartbeat;
        digitalWrite(PIN_OUT_LED_MASTER_HEARTBEAT, evenMasterHeartbeat ? HIGH : LOW);
        masterLastHeartBeatTime = millis();
        break;
      case RADIO_MSG_MASTER_MODE_DIAGNOSE:
        setCellsCountInUse(SLAVE_CELLS_COUNT);
        break;
      case RADIO_MSG_MASTER_MODE_FREESTART:
        setCellsCountInUse(1);
        break;
      case RADIO_MSG_MASTER_MODE_KO_SYSTEM_SINGLE:
        setCellsCountInUse(1);
        break;
      case RADIO_MSG_MASTER_MODE_KO_SYSTEM_DUEL:
        setCellsCountInUse(SLAVE_CELLS_COUNT);
        break;
    }
  }
}

// *******************************************************
// **
// ** CELLS
// **
// *******************************************************

void setCellsCountInUse(byte newCount) {
  cellsCountInUse = newCount;
  for (int cellId = 0; cellId < cellsCountInUse; cellId++)
    digitalWrite(PIN_LASER[cellId], HIGH);
  for (int cellId = cellsCountInUse + 1; cellId < SLAVE_CELLS_COUNT; cellId++)
    digitalWrite(PIN_LASER[cellId], LOW);
}

int readCellValue(byte cellId) {
  return analogRead(PIN_CELL[cellId]);
}

void sendCellStatus(byte cellId, CellStatusEnum cellStatus) {
  switch (cellStatus) {
    case cellStatus_free :
      Serial.print("Send : free #");
      Serial.println(cellId);
      sendRadioMessageToMaster(RADIO_MSG_SLAVE_CELL_FREE[cellId]);
      break;
    case cellStatus_obstacle:
      Serial.print("Send : obstacle #");
      Serial.println(cellId);
      sendRadioMessageToMaster(RADIO_MSG_SLAVE_CELL_OBSTACLE[cellId]);
      delay(100);
      break;
  }
}

void checkCellStatus(byte cellId) {
  int val = readCellValue(cellId);
  CellStatusEnum cellStatus = cellStatus_unknown;
  if (val > defaultCellValue[cellId] + CELL_VALUE_DELTA)
    cellStatus = cellStatus_free;
  else
    cellStatus = cellStatus_obstacle;
  if (cellStatus == lastCellStatus[cellId]) {
    // nothing changed
    return;
  }

  lastCellStatus[cellId] = cellStatus;
  sendCellStatus(cellId, cellStatus);

}

// *******************************************************
// **
// ** LOOP
// **
// *******************************************************
void loop() {
  sendRadioHeartBeatIfNeeded();
  for (int cellId = 0; cellId < cellsCountInUse; cellId++) {
    checkCellStatus(cellId);
  }
}

