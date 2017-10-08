#include <SPI.h>      // Pour la communication via le port SPI
#include <Mirf.h>     // Pour la gestion de la communication
#include <nRF24L01.h> // Pour les définitions des registres du nRF24L01
#include <MirfHardwareSpiDriver.h> // Pour la communication SPI

const int PIN_CELL = A0;
const int PIN_LED = 2;
const int HEARTBEAT_PERIOD = 500;
const int CELL_VALUE_DELTA = 30;
unsigned long lastHeartBeatTime = 0;
int defaultCellValue;

enum CellStatusEnum {
  cellStatus_unknown,
  cellStatus_obstacle,
  cellStatus_free
};

CellStatusEnum lastCellStatus = cellStatus_unknown;

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
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CELL, INPUT);
  Serial.println("Starting...");
  // Make sure the led is off
  digitalWrite(PIN_LED, LOW);
  defaultCellValue = readCellValue();
  // Switch on the led
  digitalWrite(PIN_LED, HIGH);
  Serial.print("Default value = ");
  Serial.println(defaultCellValue);
}

void sendToRadio(byte value) {
  byte message[1];
  message[0] = value;
  Mirf.send(message); // (byte *) &value);
  while (Mirf.isSending());
}

void sendHeartBeatIfNeeded() {
  unsigned long now = millis();
  if (now - lastHeartBeatTime > HEARTBEAT_PERIOD) {
    sendToRadio(1);
    lastHeartBeatTime = now;
    Serial.println("hb");
  }

}

void sendCellStatus(CellStatusEnum cellStatus) {
  switch (cellStatus) {
    case cellStatus_unknown :
      Serial.println("Send : unknown");
      sendToRadio(2);
      break;
    case cellStatus_free :
      Serial.println("Send : free");
      sendToRadio(3);
      break;
    case cellStatus_obstacle:
      Serial.println("Send : obstacle");
      sendToRadio(4);
      delay(100);
      break;
  }
}

int readCellValue() {
  return analogRead(PIN_CELL);
}

void loop() {
  sendHeartBeatIfNeeded();

  int val = readCellValue();
  CellStatusEnum cellStatus = cellStatus_unknown;
  if (val > defaultCellValue + CELL_VALUE_DELTA)
    cellStatus = cellStatus_free;
  else
    cellStatus = cellStatus_obstacle;
  if (cellStatus != lastCellStatus) {
    lastCellStatus = cellStatus;
    sendCellStatus(lastCellStatus);
  }
}

