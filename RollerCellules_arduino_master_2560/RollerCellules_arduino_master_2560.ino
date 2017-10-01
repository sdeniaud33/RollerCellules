#include <SoftwareSerial.h>
#include <SPI.h>      // Pour la communication via le port SPI
#include <Mirf.h>     // Pour la gestion de la communication
#include <nRF24L01.h> // Pour les définitions des registres du nRF24L01
#include <MirfHardwareSpiDriver.h> // Pour la communication SPI

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <SoftwareSerial.h>

#define serialBluetooth Serial1

#define PIN_SERIAL_MP3_RX 2
#define PIN_SERIAL_MP3_TX 3

#define PIN_BTN_MODE 25

#define PIN_OUT_LED_SLAVE_STATUS 11
#define PIN_OUT_LED_CELL_ALIGNED 12

#define PIN_IN_ANALOG_LDR 13

SoftwareSerial serial_mp3(PIN_SERIAL_MP3_RX, PIN_SERIAL_MP3_TX); // RX, TX


#define LCD 1
#define DEBUG true

enum modeEnum {
  modeWaitingForSlave,
  modeCalibrating,
  modeFreeStart,
  modeKoSystem
};

modeEnum currentMode = modeCalibrating;
//modeEnum currentMode = modeWaitingForSlave;

enum cellStatusEnum {
  statusUnknown,
  statusFree,
  statusObstacle
};

enum slaveStatusEnum {
  statusOnline,
  statusOffline
};

bool _running = false;
cellStatusEnum startCellStatus = statusUnknown;
cellStatusEnum arrivalCellStatus = statusUnknown;
slaveStatusEnum slaveStatus = statusOffline;

unsigned long _lastSlaveHeartbeatMillis = 0;
long slaveHeartbeatTimeToLive = 1200;
long _runningDuration;
unsigned long _runningStartTime;
#define THRESHOLD_START 500
#define THRESHOLD_END 500

#if LCD
#define I2C_ADDR 0x27 // <<—– Mettre votre adresse
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7
LiquidCrystal_I2C _lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
#endif

OneButton btnMode(PIN_BTN_MODE, true);

void setup() {
  Mirf.cePin = 9; // Broche CE sur D9
  Mirf.csnPin = 10; // Broche CSN sur D10
  Mirf.spi = &MirfHardwareSpi; // On veut utiliser le port SPI hardware
  Mirf.init(); // Initialise la bibliothèque

  Mirf.channel = 1; // Choix du canal de communication (128 canaux disponibles, de 0 à 127)
  Mirf.payload = sizeof(byte); // Taille d'un message (maximum 32 octets)
  Mirf.config(); // Sauvegarde la configuration dans le module radio

  Mirf.setTADDR((byte *) "nrf01"); // Adresse de transmission
  Mirf.setRADDR((byte *) "nrf02"); // Adresse de réception

  pinMode(PIN_OUT_LED_SLAVE_STATUS, OUTPUT);
  pinMode(PIN_OUT_LED_CELL_ALIGNED, OUTPUT);
#if DEBUG
  pinMode(PIN_IN_ANALOG_LDR, INPUT_PULLUP);
#else
  pinMode(PIN_IN_ANALOG_LDR, INPUT);
#endif
  btnMode.attachClick(onClickBtnMode);

  //    serial_mp3.begin(9600);
  serialBluetooth.begin(9600);


#if LCD
  _lcd.begin(16, 2);
  // Switch on the backlight
  _lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  _lcd.setBacklight(HIGH);
  _lcd.clear();
#else
  Serial.begin(9600);
#endif
  initNewModeDisplay();

  Serial.begin(9600);
  Serial.println("Start");
}

/*
   Inits the display for a new mode (should be invoked everytime currentMode is changed) - 1st line of the LCD
*/
void initNewModeDisplay() {
  _running = false;
#if LCD
  _lcd.clear();
  switch (currentMode) {
    case modeWaitingForSlave:
      _lcd.print("Cells - V-1.0");
      _lcd.setCursor(0, 1);
      _lcd.print("No slave in sync");
      break;

    case modeCalibrating:
      _lcd.print("CALIBRATING");
      break;
    case modeFreeStart:
      _lcd.print("FREE START");
      _lcd.setCursor(0, 1);
      _lcd.print("Ready ...");
      break;
    case modeKoSystem:
      _lcd.print("KO SYSTEM");
      _lcd.setCursor(0, 1);
      _lcd.print("Ready ...");
      break;
  }
#else
  Serial.print("New mode = ");
  switch (currentMode) {
    case modeWaitingForSlave:
      Serial.println("initializing");
      break;
    case modeCalibrating:
      Serial.println("calibrating");
      break;
    case modeFreeStart:
      Serial.println("freeStart");
      break;
    case modeKoSystem:
      Serial.println("koSystem");
      break;
  }
#endif
}

/*
   Returns a char to represent a given cell status
*/
char cellStatusToChar(cellStatusEnum cellStatus) {
  switch (cellStatus) {
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
void updateCurrentModeDisplay() {
  if ( currentMode == modeWaitingForSlave)
    return;
#if LCD
  _lcd.setCursor(0, 1);
  if (currentMode == modeCalibrating) {
    _lcd.print(cellStatusToChar(startCellStatus));
    _lcd.print(" -> ");
    _lcd.print(cellStatusToChar(arrivalCellStatus));
  }
  else {
    if (_running) {
      float f = _runningDuration;
      f = f / 1000;
      _lcd.print(f, 3);
      _lcd.print(" s");
    }
    else {
      // Leave the last duration displayed so that the runner can see its time :)
    }
  }
#else
  if (currentMode == modeCalibrating) {
    Serial.print("Calibrating ...");
    Serial.print(cellStatusToChar(startCellStatus));
    Serial.print(" -> ");
    Serial.println(cellStatusToChar(arrivalCellStatus));
  }
  else {
    if (currentMode == modeFreeStart)
      Serial.print("Freestart : ");
    else
      Serial.print("KO system :");
    if (_running) {
      Serial.print(_runningDuration);
      Serial.println(" ms");
    }
    else
      Serial.println("ready");
  }
#endif
}

/*
   This callback is invoked everytime the 'mode' button is clicked
*/
void onClickBtnMode() {
  if (currentMode == modeWaitingForSlave)
    return;
  if (currentMode == modeCalibrating)
    currentMode = modeFreeStart;
  else if (currentMode == modeFreeStart)
    currentMode = modeKoSystem;
  else if (currentMode == modeKoSystem)
    currentMode = modeCalibrating;
  else
    return;
  initNewModeDisplay();
}

/*
   Starts the chrono in 'ko system' mode
*/
void startKoSystem() {
  // Play MP3 file
  Serial.println("Start KO system");
  startChrono();
}

/*
   Starts the chrono in 'free start' mode
*/
void startFreeStart() {
  Serial.println("Start Free start");
  startChrono();
}

/*
   Starts the chrono - should not be invoked : use startFreeStart() or startKoSystem()
*/
void startChrono() {
  _running = true;
  _runningStartTime = millis();
  _runningDuration = 0;
#if LCD
  _lcd.setCursor(0, 1);
  _lcd.print("                ");
#endif
  updateCurrentModeDisplay();
}

/*
   Stops the chrono
*/
void stopChrono() {
  _running = false;
  Serial.println("Stop chrono");
  updateCurrentModeDisplay();
  float f = _runningDuration;
  f = f / 1000;
  serialBluetooth.print(f, 3);
}

void tick() {
  unsigned long now = millis();
  if (_running) {
    _runningDuration = (now - _runningStartTime) * 1.009; // 1 < x < 1.010
    updateCurrentModeDisplay();
  }
  if (currentMode == modeCalibrating) {
    updateCurrentModeDisplay();
  }
  slaveStatus = (now - _lastSlaveHeartbeatMillis) < slaveHeartbeatTimeToLive ? statusOnline : statusOffline;
  digitalWrite(PIN_OUT_LED_SLAVE_STATUS, slaveStatus == statusOnline ? HIGH : LOW);
}

/*
   Check status of the start cell
*/
void checkStartCellStatus() {
#if DEBUG
  bool startBeamReceivesLight = (digitalRead(PIN_IN_ANALOG_LDR) == LOW);
#else
  bool startBeamReceivesLight = analogRead(PIN_IN_ANALOG_LDR) > THRESHOLD_START;
#endif
  startCellStatus = (startBeamReceivesLight ? statusFree : statusObstacle);
  digitalWrite(PIN_OUT_LED_CELL_ALIGNED, startBeamReceivesLight ? HIGH : LOW);
  if ((startCellStatus == statusObstacle) && !_running) {
    if (currentMode == modeKoSystem) {
      startKoSystem();
    }
    else if (currentMode == modeFreeStart) {
      startFreeStart();
    }
  }
}


/*
   Read incoming data from bluetooth
*/
void readFromBluetooth() {
  if (currentMode == modeWaitingForSlave)
    return;

  if (!serialBluetooth.available())
    return;
  String str = serialBluetooth.readStringUntil('\n');
  Serial.print("- BLUETOOTH -");
  Serial.print(str);
  Serial.println("--");

  if (str[str.length() - 1] == '\r') {
    str.remove(str.length() - 1, 1);
  }

  if (str.equals("<start>")) {
    Serial.println("Start");
    startChrono();
  }
  else if (str.equals("<stop>")) {
    Serial.println("Stop");
    stopChrono();
  }
  else if (str.equals("<calibrate>")) {
    currentMode = modeCalibrating;
    initNewModeDisplay();
  }
  else if (str.equals("<freestart>")) {
    currentMode = modeFreeStart;
    initNewModeDisplay();
  }
  else if (str.equals("<kosystem>")) {
    currentMode = modeKoSystem;
    initNewModeDisplay();
  }
  else {
    Serial.println("Not parsed");
  }
}

void readFromRadio() {
  byte message[1];
  if (!Mirf.isSending() && Mirf.dataReady()) {
    Mirf.getData(message);
    switch (message[0]) {
      case 1 : // heartbeat
        Serial.println("<hb_slave>");
        _lastSlaveHeartbeatMillis = millis();
        if (currentMode == modeWaitingForSlave) {
          // End of init : start the calibration
          currentMode = modeCalibrating;
          initNewModeDisplay();
        }
        break;
      case 3 : // slave cell status = free
        arrivalCellStatus = statusFree;
        if (currentMode == modeCalibrating)
          updateCurrentModeDisplay();
        Serial.println("<free>");
        break;
      case 4 : // slave cell status = obstacle
        Serial.println("<obstacle>");
        arrivalCellStatus = statusObstacle;
        if (_running)
          stopChrono();
        else if (currentMode == modeCalibrating)
          updateCurrentModeDisplay();
        break;
    }
  }
}
/*
   Main loop
*/
void loop() {
  //  readFromArduinoRadio();
  readFromBluetooth();
  readFromRadio();
  tick();
  /*
    checkStartCellStatus();
    btnMode.tick();
  */
}
