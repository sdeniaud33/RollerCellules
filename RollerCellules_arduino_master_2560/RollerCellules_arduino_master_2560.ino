#include <SPI.h>      // Pour la communication via le port SPI
#include <Mirf.h>     // Pour la gestion de la communication
#include <nRF24L01.h> // Pour les définitions des registres du nRF24L01
#include <MirfHardwareSpiDriver.h> // Pour la communication SPI
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include "SimpleTimer.h"

#define serialBluetooth Serial1
#define serialMp3 Serial2

// *******************************************************
// **
// ** PIN DECLARATIONS
// **
// *******************************************************
#define PIN_OUT_LASER 8                // laser light

#define PIN_RADIO_CE 9                 // radio module : CE pin 
#define PIN_RADIO_CSN 10               // radio module : CSN pin

#define PIN_OUT_LED_SLAVE_HEARTBEAT 11 // led that will blink each time a heartbeat is received from the salve module
#define PIN_OUT_SLAVE_CELL_STATUS 12   // led that reflects the slave cell status (on = free, off = obstacle)
#define PIN_OUT_MASTER_CELL_STATUS 13  // led that reflects the master cell status (on = free, off = obstacle)
#define PIN_BTN_MODE 25                // button to switch modes
#define PIN_BTN_RESET 26               // hardware only : not used directly in the code 
#define PIN_RADIO_MISO 50              // hardware only : not used directly in the code
#define PIN_RADIO_MOSI 51              // hardware only : not used directly in the code
#define PIN_RADIO_SCK 52               // hardware only : not used directly in the code

#define PIN_IN_ANALOG_LDR A0           // Analog read to get the value of the LDR

// *******************************************************
// **
// ** OTHER DECLARATIONS
// **
// *******************************************************

const int MP3_ID_ON_YOUR_MARKS_SET = 2;
const int MP3_ID_GO = 1;
DFRobotDFPlayerMini myDFPlayer;

bool evenSlaveHeartbeat = false;       // even / odd flag toggled each time a heartbeat is received from the slave module
SimpleTimer koSystemStartTimer;

// Enum for the status of the module (master module)
enum modeEnum {
  modeWaitingForMp3,        // the MP3 player is initializing
  modeWaitingForSlave,      // waiting for the first heartbeat from then slave module
  modeDiagnose,             // diagnosis : mainly show cells alignments
  modeFreeStart,            // 'free start' mode
  modeKoSystem              // 'k.o system' mode
};

modeEnum currentMode;

enum cellStatusEnum {
  statusUnknown,
  statusFree,
  statusObstacle
};

enum slaveStatusEnum {
  statusOnline,
  statusOffline
};

bool isRunning = false;
bool isStartingKoSsytem = false;
cellStatusEnum lastMasterCellStatus = statusUnknown;
cellStatusEnum masterCellStatus = statusUnknown;
cellStatusEnum slaveCellStatus = statusUnknown;
slaveStatusEnum slaveStatus = statusOffline;

unsigned long lastSlaveHeartbeatMillis = 0;
long slaveHeartbeatTimeToLive = 1200;
long runningDuration;
unsigned long runningStartTime;
const int CELL_VALUE_DELTA = 30;
int defaultCellValue;

#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3
LiquidCrystal_I2C lcd(I2C_ADDR, 2, 1, 0, 4, 5, 6, 7);

OneButton btnMode(PIN_BTN_MODE, false);

// *******************************************************
// **
// ** SETUP
// **
// *******************************************************
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing ...");

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

  Mirf.setTADDR((byte *) "nrf01");
  Mirf.setRADDR((byte *) "nrf02");

  // ----- Initialization of the bluetooth module
  serialBluetooth.begin(9600);

  // ----- Initialization of the cells
  pinMode(PIN_OUT_LED_SLAVE_HEARTBEAT, OUTPUT);
  pinMode(PIN_OUT_SLAVE_CELL_STATUS, OUTPUT);
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
  if (!myDFPlayer.begin(serialMp3)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true);
  }
  myDFPlayer.enableDAC();
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30

  // ----- Initialization other stuff
  btnMode.attachClick(onClickBtnMode);

  currentMode = modeDiagnose;
  initNewModeDisplay();

  Serial.begin(9600);
  Serial.println("Ready");

  Serial.print("Default value = ");
  Serial.println(defaultCellValue);

}

/*
   Inits the display for a new mode (should be invoked everytime currentMode is changed) - 1st line of the LCD
*/
void initNewModeDisplay() {
  isStartingKoSsytem = false;
  isRunning = false;
  lcd.clear();
  switch (currentMode) {
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
      break;
    case modeFreeStart:
      lcd.print(" - FREE START - ");
      lcd.setCursor(0, 1);
      lcd.print("Ready ...");
      break;
    case modeKoSystem:
      lcd.print(" - K.O SYSTEM - ");
      lcd.setCursor(0, 1);
      lcd.print("Ready ...");
      break;
  }
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
  if ( currentMode == modeWaitingForMp3)
    return;

  lcd.setCursor(0, 1);
  if (currentMode == modeDiagnose) {
    lcd.print(cellStatusToChar(masterCellStatus));
    lcd.print(" -> ");
    lcd.print(cellStatusToChar(slaveCellStatus));
  }
  else {
    if (isRunning) {
      float f = runningDuration;
      f = f / 1000;
      lcd.print(f, 3);
      lcd.print(" s");
    }
    else {
      // Leave the last duration displayed so that the runner can see its time :)
    }
  }
}

/*
   This callback is invoked everytime the 'mode' button is clicked
*/
void onClickBtnMode() {
  Serial.println("Click");
  if (currentMode == modeWaitingForMp3)
    return;
  if (currentMode == modeWaitingForSlave)
    return;

  if (currentMode == modeDiagnose)
    currentMode = modeFreeStart;
  else if (currentMode == modeFreeStart)
    currentMode = modeKoSystem;
  else if (currentMode == modeKoSystem)
    currentMode = modeDiagnose;
  else
    return;
  initNewModeDisplay();
}

/*
   Starts the chrono in 'ko system' mode
*/
void startKoSystem() {
  if (isStartingKoSsytem)
    return;
  if (isRunning)
    return;
  isStartingKoSsytem = true;
  Serial.println("Start KO system");
  // Play the "on your marks .. set ..." mp3 file
  myDFPlayer.play(MP3_ID_ON_YOUR_MARKS_SET);
  lcd.setCursor(0, 1);
  lcd.print("On your marks ...                 ");
  koSystemStartTimer.setTimeout(4000, startKoSystemStepSet);
}

void startKoSystemStepSet() {
  if (!isStartingKoSsytem) {
    myDFPlayer.stop();
    return;
  }
  lcd.setCursor(0, 1);
  lcd.print("SET ...                           ");
  koSystemStartTimer.setTimeout(1000, startKoSystemStepGo);
}

void startKoSystemStepGo() {
  if (!isStartingKoSsytem) {
    myDFPlayer.stop();
    return;
  }
  // Play the "go !!!" mp3 file
  myDFPlayer.play(MP3_ID_GO);
  startChrono();
}

/*
   Starts the chrono in 'free start' mode
*/
void startFreeStart() {
  if (isStartingKoSsytem)
    return;
  Serial.println("Start Free start");
  startChrono();
}

/*
   Starts the chrono - should not be invoked : use startFreeStart() or startKoSystem() instead
*/
void startChrono() {
  isStartingKoSsytem = false;
  isRunning = true;
  runningStartTime = millis();
  runningDuration = 0;
  lcd.setCursor(0, 1);
  lcd.print("                ");
  updateCurrentModeDisplay();
}

/*
   Stops the chrono
*/
void stopChrono() {
  isRunning = false;
  Serial.println("Stop chrono");
  updateCurrentModeDisplay();
  float f = runningDuration;
  f = f / 1000;
  serialBluetooth.print(f, 3);
}

void tick() {
  unsigned long now = millis();
  if (isRunning) {
    runningDuration = (now - runningStartTime) * 1.009; // 1 < x < 1.010
    updateCurrentModeDisplay();
  }
  if (currentMode == modeDiagnose) {
    updateCurrentModeDisplay();
  }
  slaveStatus = (now - lastSlaveHeartbeatMillis) < slaveHeartbeatTimeToLive ? statusOnline : statusOffline;
}

int readCellValue() {
  return analogRead(PIN_IN_ANALOG_LDR);
}

/*
   Check status of the start cell
*/
void checkMasterCellStatus() {
  int val = readCellValue();
  masterCellStatus = statusUnknown;
  if (val > defaultCellValue + CELL_VALUE_DELTA)
    masterCellStatus = statusFree;
  else
    masterCellStatus = statusObstacle;

  if (masterCellStatus == lastMasterCellStatus) {
    // no change
    return;
  }

  lastMasterCellStatus = masterCellStatus;

  if (isRunning) {
    // should not happen .... never knows
    return;
  }
  if (masterCellStatus == statusObstacle) {
    // The beam has been interrupted
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
  if (currentMode == modeWaitingForMp3)
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
    if (currentMode == modeFreeStart)
      startFreeStart();
    else if (currentMode == modeKoSystem)
      startKoSystem();
  }
  else if (str.equals("<stop>")) {
    Serial.println("Stop");
    stopChrono();
  }
  else if (str.equals("<calibrate>")) {
    currentMode = modeDiagnose;
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

/*
   Read incoming data from the radio module (receive data from the slave module)
*/
void readFromRadio() {
  byte message[1];
  if (!Mirf.isSending() && Mirf.dataReady()) {
    Mirf.getData(message);
    switch (message[0]) {
      case 1 : // heartbeat
        Serial.println("<hb_slave>");
        evenSlaveHeartbeat = !evenSlaveHeartbeat;
        digitalWrite(PIN_OUT_LED_SLAVE_HEARTBEAT, evenSlaveHeartbeat ? HIGH : LOW);
        lastSlaveHeartbeatMillis = millis();
        if (currentMode == modeWaitingForSlave) {
          // End of init : start the calibration
          currentMode = modeDiagnose;
          initNewModeDisplay();
        }
        break;
      case 3 : // slave cell status = free
        slaveCellStatus = statusFree;
        digitalWrite(PIN_OUT_SLAVE_CELL_STATUS, HIGH);
        if (currentMode == modeDiagnose)
          updateCurrentModeDisplay();
        Serial.println("<free>");
        break;
      case 4 : // slave cell status = obstacle
        Serial.println("<obstacle>");
        digitalWrite(PIN_OUT_SLAVE_CELL_STATUS, LOW);
        slaveCellStatus = statusObstacle;
        if (isRunning)
          stopChrono();
        else if (currentMode == modeDiagnose)
          updateCurrentModeDisplay();
        break;
    }
  }
}

void printDetailFromMp3Player(uint8_t type, int value) {
  Serial.print("Output from MP3 player : ");
  switch (type) {
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
      switch (value) {
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
/*
   Main loop
*/
void loop() {
  //  readFromArduinoRadio();
  readFromBluetooth();
  readFromRadio();
  tick();

  if (myDFPlayer.available()) {
    printDetailFromMp3Player(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  checkMasterCellStatus();
  //btnMode.tick();
  koSystemStartTimer.run();
}
