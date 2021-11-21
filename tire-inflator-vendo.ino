#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
HardwareSerial &gsmSerial = Serial3;




//----------------------------- CONFIGURATIONS ------------------------------------

// EEPROM
const byte numOfCoinsAddress = 0;
const byte ownerNumberAddress = 16;

// Keypad
const byte numRows = 4;
const byte numCols = 4;
const byte rowPins[numRows] = {6, 5, 4, 3};
const byte colPins[numCols] = {10, 9, 8, 7};

// Keypad
char keys[numRows][numCols] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, numRows, numCols );

// LCD
const byte lcdI2cAddress = 0x27;
const byte lcdNumRows = 20;
const byte lcdNumCols = 4;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(lcdI2cAddress, lcdNumRows, lcdNumCols);

// Pressure sensor
const byte pressureInputPin = A1;

// Coin slot
const int coinPin = 2;

// buzzer
const byte buzzerPin = 12;

// Reset button
const byte resetPin = 11;

// compressor/relay
const byte relayPin = 16;

//----------------------------- END CONFIGURATIONS ---------------------------------




//----------------------------- GLOBAL VARIABLES ----------------------------------

// Pressure constants
const float b = 625.0;
const float c = 103.0;
const float x = 232.0603804;

// Standard bike pressure
const byte bikeStandardFrontPressure = 40;
const byte bikeStandardRearPressure = 70;
bool isBikeTireFront = false;

// Standard motor pressure
// front
const byte motorStandardFrontPressure = 28;
// rear
const byte motorStandardRearPressure = 35;
bool isMotorTireFront = false;

// Standard CAR pressure
const byte carStandardPressure = 36;
char carType[8];
bool isLoaded = false;
bool isFront = false;
// Sedan
// non-loaded
const byte sedanNonloadedFrontPressure = 33;
const byte sedanNonloadedRearPressure = 33;
// loaded
const byte sedanLoadedFrontPressure = 35;
const byte sedanLoadedRearPressure = 38;

// SUV
// non-loaded
const byte suvNonloadedFrontPressure = 29;
const byte suvNonloadedRearPressure = 32;
// loaded
const byte suvLoadedFrontPressure = 35;
const byte suvLoadedRearPressure = 36;

// Pick-up truck
// non-loaded
const byte pickupNonloadedFrontPressure = 35;
const byte pickupNonloadedRearPressure = 35;
// loaded
const byte pickupLoadedFrontPressure = 38;
const byte pickupLoadedRearPressure = 44;


// Pressure calculations
float finalPressure = 0.0;
float tirePressure = 0.0;
float currentPressure = 0.0;
// smoothing/averaging
const byte numOfPressureSamples = 30; // if you change this
const float numOfPressureSamplesInFloat = 30; // change this also
int readIndex = 0;
float total = 0;
float pressureSamples[numOfPressureSamples];
bool hasStartedGettingPressureSamples = false;
unsigned long lastSampleRead = 0;
unsigned int sampleTimeout = 100;
// lcd x-coordinate for finalPressure
byte finalPressurePlacementX = 0;



// injecting air
unsigned long lastPressureRead = 0;
unsigned int pressureReadTimeout = 100;

// screen
volatile char currentScreen[32] = "chooseMode";
char prevScreen[32] = "";
char inputPressure[8];
char inputOwnerNumber[16];
char forbiddenInputKeys[6] = {'A', 'B', 'C', 'D', '#'};

// mode and vehicle type
char mode[8];
char vehicleType[16];

// restart screen (back to choose mode)
const int restartTimeout = 3000;
unsigned long lastRestart = 0;
bool restartRoutine = false;


// coin
volatile bool hasInsertedCoin = false;
volatile bool hasReachedCoinLimit = false;
const int numOfCoinsLimit = 5;
const int coinCountLimit = 5;

// sms
bool isNotified = false;
char reachedCoinLimitMessage[64];
char currentIncomeMessage[64];


// GSM command timing
unsigned long gsmStartedAt = 0;
unsigned int gsmTimeout = 1000;

// GSM Serial parser
bool isGsmResponseReady = false;
bool isFullResponseReady = false;
const byte numChars = 128;
char gsmResponse[numChars];
char tempGsmResponse[numChars];

// used in every gsm command
unsigned long startedAt = 0;


// Get current time routine
char currentRoutine[18] = "reachedCoinLimit";

// Get message routine
bool hasBeenSetToTextMode = false;
const byte maxRetryCount = 10;
byte retryCount = 0;

// reply/sms routine
bool hasStartedSendingSms = false;
char currentCommand[8];
char prevCommand[8];
char reply[32];
unsigned int commandInterval = 1000;
const byte maxReplyCount = 2;
byte replyCount = 0;

// Parse command routine
bool isOwner = false;
char receivedCommand[8];

// Get current time routine
bool isGettingTime = false;


// Initialize GSM
bool isDoneCheckingStatus = false;
bool isCheckingNetworkStatus = false;


// reset owner number
unsigned long timePressed = 0;
unsigned long timeReleased = 0;
const int timeout = 5000;
bool hasStartedPressing = false;


// main timing
unsigned long timeElapsed = 0;

byte tirePressureSamples = 0;
byte maxTirePressureSampleCount = 2;
//-------------------------- END GLOBAL VARIABLES --------------------------------




void setup() {
  Serial.begin(9600);
  setReachedCoinLimitMessage();
  clearPressureSamples();
  initializeLcd();
  configurePins();
  initializeGsm();
}

void loop() {
  timeElapsed = millis();
  runBackgroundServices();
  getTirePressure();
  resetOwnerNumber();
  keypadEventListener();
  currentScreenProvider();
  injectAir();
}




//------------------------- INITIALIZATION FUNCTIONS -----------------------------

// LCD
void initializeLcd() {
  lcd.init();
  lcd.backlight();
}

// GSM
void initializeGsm() {
  gsmSerial.begin(9600);
  Serial.println(F("Connecting to cellular network"));

  while (!isDoneCheckingStatus) {
    if (millis() - gsmStartedAt >= 1500) {
      gsmStartedAt = millis();
      if (!isCheckingNetworkStatus) {
        Serial.println(F("Connecting to cellular network"));
        gsmSerial.print("AT+CREG?\r\n");
        isCheckingNetworkStatus = true;
      } else {
        isCheckingNetworkStatus = false;
      }
    }

    readLine(3);
    getNetworkStatus();
  }
}
void getNetworkStatus() {
  if (!isGsmResponseReady) {
    return;
  }

  char *response;
  char *mode;
  char *networkStatus;

  response = strstr(gsmResponse, "+CREG: ");
  mode = strtok(response, ",");
  Serial.print(F("GSM response >> "));
  Serial.println(response);
  if (mode != NULL) {
    networkStatus = strtok(NULL, ",");
    if (networkStatus != NULL) {
      if (*networkStatus == '1') {
        isDoneCheckingStatus = true;
        gsmStartedAt = 0;
        Serial.println(F("Successfully connected to network!"));
        return;
      }
    }
  }

  isGsmResponseReady = false;
}

// Input/Output pins
void configurePins() {
  pinMode(pressureInputPin, INPUT);
  pinMode(coinPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  // attach interrupt to coin slot
  attachInterrupt(digitalPinToInterrupt(coinPin), insertCoinInterrupt, RISING);

  // initially set buzzer pin to HIGH
  digitalWrite(buzzerPin, HIGH);

  // relay pin OFF state
  digitalWrite(relayPin, HIGH);
}

//------------------------- END INITIALIZATION FUNCTIONS --------------------------




//------------------------- KEYPAD EVENT LISTENER ---------------------------------

void keypadEventListener() {
  char key = keypad.getKey();

  // Set owner number
  if (!strcmp(currentScreen, "setOwnerNumber")) {
    if (key == 'C') {
      if (strlen(inputOwnerNumber) != 10) {
        return;
      }

      char finalOwnerNumber[16];
      char existingOwnerNumber[16];
      sprintf(finalOwnerNumber, "+63%s", inputOwnerNumber);
      EEPROM.put(ownerNumberAddress, finalOwnerNumber);
      EEPROM.get(ownerNumberAddress, existingOwnerNumber);
      Serial.print(F("Owner number successfully set >> "));
      Serial.println(existingOwnerNumber);
      strcpy(currentScreen, "chooseMode");
      clearInputOwnerNumber();
      return;
    }

    if (key == 'B') {
      strcpy(currentScreen, "chooseMode");
      return;
    }

    if (key == '*') {
      clearInputOwnerNumber();
      return;
    }

    if (strchr(forbiddenInputKeys, key) != NULL) {
      return;
    }

    byte len = strlen(inputOwnerNumber);
    if (len >= 10) {
      return;
    }
    inputOwnerNumber[len] = key;
    inputOwnerNumber[len + 1] = NULL;
    // len + 3 (offset for "+63" characters)
    lcd.setCursor(len + 3, 1);
    if (inputOwnerNumber[len] != NULL) {
      lcd.print(inputOwnerNumber[len]);
      lcd.blink();
    }
  }

  // Choose mode
  if (!strcmp(currentScreen, "chooseMode")) {
    if (key == 'A') {
      strcpy(mode, "auto");
      strcpy(currentScreen, "plugInNozzle");
      return;
    }

    if (key == 'B') {
      strcpy(mode, "manual");
      strcpy(currentScreen, "plugInNozzle");
      return;
    }
  }

  // Input pressure
  if (!strcmp(currentScreen, "inputPressure")) {
    // confirm
    if (key == 'C') {
      finalPressure = atoi(inputPressure) + tirePressure;
      Serial.print(F("Final pressure = tire pressure + desired pressure >> "));
      Serial.print(tirePressure);
      Serial.print(F(" + "));
      Serial.print(atoi(inputPressure));
      Serial.print(F(" = "));
      Serial.println(finalPressure);

      if (finalPressure == 0) {
        return;
      }
      strcpy(currentScreen, "desiredPressure");
      return;
    }
    // clear
    if (key == '#') {
      clearInputPressure();
      return;
    }
    // back
    if (key == '*') {
      strcpy(currentScreen, "chooseMode");
      clearInputPressure();
      return;
    }

    if (strchr(forbiddenInputKeys, key) != NULL) {
      return;
    }

    byte len = strlen(inputPressure);
    if (len >= 2) {
      return;
    }
    inputPressure[len] = key;
    inputPressure[len + 1] = NULL;

    lcd.setCursor(len, 1);
    lcd.print(inputPressure[len]);
    lcd.blink();
  }

  // Plug in nozzle
  if (!strcmp(currentScreen, "plugInNozzle")) {
    // confirm
    if (key == 'C') {
      // read tire pressure
      hasStartedGettingPressureSamples = true;
      strcpy(currentScreen, "readingPressure");
      return;
    }
    //  back
    if (key == '*') {
      strcpy(currentScreen, "chooseMode");
      return;
    }
  }

  // display pressure
  if (!strcmp(currentScreen, "displayPressure")) {
    if (key == 'C') {
      if (!strcmp(mode, "auto")) {
        strcpy(currentScreen, "vehicleType");
        return;
      } else {
        strcpy(currentScreen, "inputPressure");
      }
    }
  }

  // Choose vehicle type
  if (!strcmp(currentScreen, "vehicleType")) {
    if (key == 'A') {
      strcpy(vehicleType, "bike");
      strcpy(currentScreen, "bikeFrontOrRear");
      return;
    }
    if (key == 'B') {
      strcpy(vehicleType, "motor");
      strcpy(currentScreen, "motorFrontOrRear");
      return;
    }
    if (key == 'C') {
      strcpy(vehicleType, "car");
      strcpy(currentScreen, "carType");
      return;
    }
  }

  // bike front or rear tire
  if (!strcmp(currentScreen, "bikeFrontOrRear")) {
    if (key == 'A') {
      isBikeTireFront = true;
      calcFinalPressure(getBikeStandardPressure());
      strcpy(currentScreen, "recommendedPressure");
      return;
    }
    if (key == 'B') {
      isBikeTireFront = false;
      strcpy(currentScreen, "recommendedPressure");
      calcFinalPressure(getBikeStandardPressure());
      return;
    }
  }

  // motor front or rear tire
  if (!strcmp(currentScreen, "motorFrontOrRear")) {
    if (key == 'A') {
      isMotorTireFront = true;
      calcFinalPressure(getMotorStandardPressure());
      strcpy(currentScreen, "recommendedPressure");
      return;
    }
    if (key == 'B') {
      isMotorTireFront = false;
      strcpy(currentScreen, "recommendedPressure");
      calcFinalPressure(getMotorStandardPressure());
      return;
    }
  }

  // Choose car type
  if (!strcmp(currentScreen, "carType")) {
    if (key == 'A') {
      strcpy(carType, "sedan");
      strcpy(currentScreen, "isCarLoaded");
      return;
    }
    if (key == 'B') {
      strcpy(carType, "suv");
      strcpy(currentScreen, "isCarLoaded");
      return;
    }
    if (key == 'C') {
      strcpy(carType, "pickup");
      strcpy(currentScreen, "isCarLoaded");
      return;
    }
  }
  // is car loaded
  if (!strcmp(currentScreen, "isCarLoaded")) {
    if (key == 'A') {
      isLoaded = true;
      strcpy(currentScreen, "isFrontOrRear");
      return;
    }
    if (key == 'B') {
      isLoaded = false;
      strcpy(currentScreen, "isFrontOrRear");
      return;
    }
  }
  // front or rear
  if (!strcmp(currentScreen, "isFrontOrRear")) {
    if (key == 'A') {
      isFront = true;
      calcFinalPressure(getCarStandardPressure());
      strcpy(currentScreen, "recommendedPressure");
      return;
    }
    if (key == 'B') {
      isFront = false;
      calcFinalPressure(getCarStandardPressure());
      strcpy(currentScreen, "recommendedPressure");
      return;
    }
  }

  // recommend pressure
  if (!strcmp(currentScreen, "recommendedPressure")) {
    // plus
    if (key == 'A') {
      finalPressure++;
      lcd.setCursor(finalPressurePlacementX, 1);
      lcd.print(finalPressure);
      return;
    }
    // minus
    if (key == 'B') {
      finalPressure--;
      lcd.setCursor(finalPressurePlacementX, 1);
      lcd.print(finalPressure);
      return;
    }
    // confirm
    if (key == 'C') {
      strcpy(currentScreen, "insertCoin");
      Serial.print(F("Final pressure is "));
      Serial.println(finalPressure);
      return;
    }
    // cancel
    if (key == '*') {
      strcpy(currentScreen, "chooseMode");
      return;
    }
  }

  // desired pressure
  if (!strcmp(currentScreen, "desiredPressure")) {
    // confirm
    if (key == 'C') {
      strcpy(currentScreen, "insertCoin");
      Serial.print(F("Final pressure is "));
      Serial.println(finalPressure);
      return;
    }
    // cancel
    if (key == '*') {
      strcpy(currentScreen, "chooseMode");
      return;
    }
  }

  // Done
  if (!strcmp(currentScreen, "pressureStatus")) {
    if (key == '*') {
      restart();
    }
  }
}

//------------------------- END KEYPAD EVENT LISTENER------------------------------




//-------------------------- SCREEN PROVIDER -------------------------------------

void currentScreenProvider() {
  if (!strcmp(currentScreen, "setOwnerNumber") && strcmp(prevScreen, "setOwnerNumber")) {
    lcd.clear();
    setOwnerNumberScreen();
  }

  if (!strcmp(currentScreen, "chooseMode") && strcmp(prevScreen, "chooseMode")) {
    lcd.clear();
    chooseModeScreen();
  }

  if (!strcmp(currentScreen, "plugInNozzle") && strcmp(prevScreen, "plugInNozzle")) {
    lcd.clear();
    lcd.noBlink();
    plugInNozzleScreen();
  }

  if (!strcmp(currentScreen, "readingPressure") && strcmp(prevScreen, "readingPressure")) {
    lcd.clear();
    readingPressureScreen();
  }

  if (!strcmp(currentScreen, "displayPressure") && strcmp(prevScreen, "displayPressure")) {
    lcd.clear();
    displayPressureScreen();
  }

  if (!strcmp(currentScreen, "carType") && strcmp(prevScreen, "carType")) {
    lcd.clear();
    carTypeScreen();
  }
  if (!strcmp(currentScreen, "isCarLoaded") && strcmp(prevScreen, "isCarLoaded")) {
    lcd.clear();
    isCarLoadedScreen();
  }
  if (!strcmp(currentScreen, "isFrontOrRear") && strcmp(prevScreen, "isFrontOrRear")) {
    lcd.clear();
    isFrontOrRearScreen();
  }


  if (!strcmp(currentScreen, "vehicleType") && strcmp(prevScreen, "vehicleType")) {
    lcd.clear();
    vehicleTypeScreen();
  }

  if (!strcmp(currentScreen, "bikeFrontOrRear") && strcmp(prevScreen, "bikeFrontOrRear")) {
    lcd.clear();
    bikeFrontOrRearScreen();
  }

  if (!strcmp(currentScreen, "motorFrontOrRear") && strcmp(prevScreen, "motorFrontOrRear")) {
    lcd.clear();
    motorFrontOrRearScreen();
  }

  if (!strcmp(currentScreen, "inputPressure") && strcmp(prevScreen, "inputPressure")) {
    lcd.clear();
    inputPressureScreen();
  }

  if (!strcmp(currentScreen, "recommendedPressure") && strcmp(prevScreen, "recommendedPressure")) {
    lcd.clear();
    recommendedPressureScreen();
  }

  if (!strcmp(currentScreen, "desiredPressure") && strcmp(prevScreen, "desiredPressure")) {
    lcd.clear();
    lcd.noBlink();
    desiredPressureScreen();
  }

  if (!strcmp(currentScreen, "insertCoin") && strcmp(prevScreen, "insertCoin")) {
    lcd.clear();
    insertCoinScreen();
  }

  if (!strcmp(currentScreen, "pressureStatus") && strcmp(prevScreen, "pressureStatus")) {
    lcd.clear();
    pressureStatusScreen();
  }

  if (!strcmp(currentScreen, "done") && strcmp(prevScreen, "done")) {
    lcd.clear();
    doneScreen();
  }

  strcpy(prevScreen, currentScreen);
}

//-------------------------- END SCREEN PROVIDER ---------------------------------




//------------------------------- SCREENS ----------------------------------------

// Choose mode
void chooseModeScreen() {
  lcd.print("Choose mode:");
  lcd.setCursor(0, 1);
  lcd.print("(A) Automatic");
  lcd.setCursor(0, 2);
  lcd.print("(B) Manual");
}

// Set owner number
void setOwnerNumberScreen() {
  lcd.print("Set owner number");
  lcd.setCursor(0, 1);
  lcd.print("+63");
  lcd.setCursor(0, 2);
  lcd.print("Press * to clear");
  lcd.setCursor(0, 3);
  lcd.print("Press C to confirm");
}
void clearInputOwnerNumber() {
  inputOwnerNumber[0] = '\0';
  lcd.setCursor(3, 1);
  lcd.print("                 ");
  lcd.noBlink();
}

// Plug in nozzle
void plugInNozzleScreen() {
  lcd.setCursor(1, 0);
  lcd.print("Plug in the nozzle");
  lcd.setCursor(1, 2);
  lcd.print("(C) Confirm");
  lcd.setCursor(1, 3);
  lcd.print("(*) Back");
}

// Reading pressure
void readingPressureScreen() {
  lcd.setCursor(2, 0);
  lcd.print("Reading pressure");
  lcd.setCursor(2, 1);
  lcd.print("inside the tire");
  lcd.setCursor(2, 3);
  lcd.print("Please wait ...");
}

// Remaining/current pressure inside the tire
void displayPressureScreen() {
  int tirePressureToInt = static_cast<int>(tirePressure);

  lcd.setCursor(1, 0);
  lcd.print("Pressure inside the");
  lcd.setCursor(1, 1);
  lcd.print("tire is");
  lcd.setCursor(9, 1);
  lcd.print(tirePressureToInt);

  // set position of the "PSI" word
  if (tirePressureToInt >= 100) {
    lcd.setCursor(13, 1);
  }
  if (tirePressureToInt < 100 && tirePressureToInt >= 10) {
    lcd.setCursor(12, 1);
  }
  if (tirePressureToInt < 10) {
    lcd.setCursor(11, 1);
  }
  lcd.print("PSI");

  lcd.setCursor(1, 3);
  lcd.print("(C) Confirm");
}

// Vehicle type
void vehicleTypeScreen() {
  lcd.print("Choose vehicle type:");
  lcd.setCursor(0, 1);
  lcd.print("(A) Bike");
  lcd.setCursor(0, 2);
  lcd.print("(B) Motor");
  lcd.setCursor(0, 3);
  lcd.print("(C) Car");
}

// Motor front/rear
void motorFrontOrRearScreen() {
  lcd.print("Front or rear?");
  lcd.setCursor(0, 1);
  lcd.print("(A) Front");
  lcd.setCursor(0, 2);
  lcd.print("(B) Rear");
}

// Bike front/rear
void bikeFrontOrRearScreen() {
  lcd.print("Front or rear?");
  lcd.setCursor(0, 1);
  lcd.print("(A) Front");
  lcd.setCursor(0, 2);
  lcd.print("(B) Rear");
}

// Car type
void carTypeScreen() {
  lcd.setCursor(0, 0);
  lcd.print("Select type of car:");
  lcd.setCursor(0, 1);
  lcd.print("(A) Sedan");
  lcd.setCursor(0, 2);
  lcd.print("(B) SUV");
  lcd.setCursor(0, 3);
  lcd.print("(C) Pickup Truck");
}
// car loaded/non-loaded
void isCarLoadedScreen() {
  lcd.setCursor(0, 0);
  lcd.print("Load type:");
  lcd.setCursor(0, 1);
  lcd.print("(A) Loaded");
  lcd.setCursor(0, 2);
  lcd.print("(B) Nonloaded");
}
// front or rear
void isFrontOrRearScreen() {
  lcd.setCursor(0, 0);
  lcd.print("Front or rear?");
  lcd.setCursor(0, 1);
  lcd.print("(A) Front");
  lcd.setCursor(0, 2);
  lcd.print("(B) Rear");
}

// Input pressure
void inputPressureScreen() {
  lcd.print("Enter pressure (PSI)");
  lcd.setCursor(0, 2);
  lcd.print("(C) Confirm");
  lcd.setCursor(0, 3);
  lcd.print("(*) Back");
  lcd.setCursor(11, 3);
  lcd.print("(#) Clear");
}
// clear pressure input
void clearInputPressure() {
  inputPressure[0] = '\0';
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.noBlink();
}

// recommended pressure (automatic)
void recommendedPressureScreen() {
  // holds "is" word x-coordinate
  byte isWordPlacementX = 0;
  // holds finalPressure value x-coordinate
  byte psiFinalPlacementX = 0;

  lcd.setCursor(0, 0);
  lcd.print("Recommended PSI for");
  lcd.setCursor(0, 1);
  // vehicle type
  if (!strcmp(vehicleType, "car")) {
    lcd.print(carType);
  } else {
    lcd.print(vehicleType);
  }

  // "is" word placement
  // motor
  if (!strcmp(vehicleType, "motor")) {
    isWordPlacementX = 6;
  }
  // bike
  if (!strcmp(vehicleType, "bike")) {
    isWordPlacementX = 5;
  }
  // car
  if (!strcmp(vehicleType, "car")) {
    if (!strcmp(carType, "sedan")) {
      isWordPlacementX = 6;
    }
    if (!strcmp(carType, "suv")) {
      isWordPlacementX = 4;
    }
    if (!strcmp(carType, "pickup")) {
      isWordPlacementX = 7;
    }
  }
  lcd.setCursor(isWordPlacementX, 1);
  lcd.print("is");

  // final pressure placement on lcd (x-coordinate)
  psiFinalPlacementX = isWordPlacementX + 3;

  // set global variable
  // to current finalPressure's x-coordinate
  finalPressurePlacementX = psiFinalPlacementX;

  // plus
  lcd.setCursor(0, 2);
  lcd.print("(A) Plus");
  // confirm
  lcd.setCursor(9, 2);
  lcd.print("(C) Confirm");
  // minus
  lcd.setCursor(0, 3);
  lcd.print("(B) Minus");
  // cancel
  lcd.setCursor(10, 3);
  lcd.print("(*) Cancel");

  lcd.setCursor(psiFinalPlacementX, 1);

  int finalPressureToInt = static_cast<int>(finalPressure);
  lcd.print(finalPressureToInt);
}

// desired pressure (manual)
void desiredPressureScreen() {
  if (!strcmp(mode, "manual")) {
    int finalPressureToInt = atoi(inputPressure);

    lcd.setCursor(0, 0);
    lcd.print("Desired pressure is");
    lcd.setCursor(0, 1);
    lcd.print(finalPressureToInt);

    // "PSI" word placement
    lcd.setCursor(strlen(inputPressure) + 1, 1);
    lcd.print("PSI");

    lcd.setCursor(0, 2);
    lcd.print("(C) Confirm");
    lcd.setCursor(0, 3);
    lcd.print("(*) Cancel");
  }
}

// insert coin
void insertCoinScreen() {
  lcd.setCursor(1, 1);
  lcd.print("Insert 5 peso coin");
}

// Air pressure status
void pressureStatusScreen() {
  lcd.setCursor(3, 0);
  lcd.print("Injecting air");
  lcd.setCursor(2, 1);
  lcd.print("Please wait ...");
}
// Display current pressure
void displayCurrentPressure() {
  int currentPressureToInt = static_cast<int>(currentPressure);
  lcd.setCursor(0, 3);
  lcd.print("Current PSI ");
  lcd.print(currentPressureToInt);
}

// Done
void doneScreen() {
  lcd.setCursor(0, 1);
  lcd.print("Done injecting air!");
}

//-------------------------------- END SCREENS ------------------------------------




//-------------------------------- RESTART/RESET ----------------------------------

// Restart screen back to choose mode
void restart() {
  strcpy(currentScreen, "chooseMode");
  mode[0] = NULL;
  vehicleType[0] = NULL;
  prevScreen[0] = NULL;
  tirePressure = 0;
  finalPressure = 0;
  currentPressure = 0;
  total = 0;
  clearPressureSamples();
  clearInputPressure();
}
// handles reset owner number button
void resetOwnerNumber() {
  bool _isPressed = false;
  _isPressed = digitalRead(resetPin);

  // pressed
  if (!_isPressed && !hasStartedPressing) {
    timePressed = timeElapsed;
    hasStartedPressing = true;
  }

  // not pressed
  if (_isPressed && hasStartedPressing) {
    timeReleased = timeElapsed;
    hasStartedPressing = false;
    if (timeReleased - timePressed >= timeout) {
      if (!strcmp(currentScreen, "chooseMode")) {
        strcpy(currentScreen, "setOwnerNumber");
      }
    }
  }
}


//------------------------------- END RESTART/RESET -------------------------------




//------------------------------- INSERT COIN -------------------------------------

void insertCoinInterrupt() {
  if (strcmp(currentScreen, "insertCoin")) {
    return;
  }

  hasInsertedCoin = true;

  byte numOfCoins;
  EEPROM.get(numOfCoinsAddress, numOfCoins);

  // increment coin count
  numOfCoins += 1;

  // reached coin limit
  // reset
  if (numOfCoins == coinCountLimit) {
    // send sms
    hasReachedCoinLimit = true;
    // reset
    EEPROM.update(numOfCoinsAddress, 0);
    return;
  }

  // has not yet reached coin limit
  // save to EEPROM
  EEPROM.update(numOfCoinsAddress, numOfCoins);

  Serial.print(F("Current coin count >> "));
  Serial.println(numOfCoins);
}

//------------------------------ END INSERT COIN-----------------------------------




//------------------------------- PRESSURE ----------------------------------------


void calcFinalPressure(byte standardPressure) {
  finalPressure = standardPressure - tirePressure;
  Serial.print("Final pressure is ");
  Serial.print(standardPressure);
  Serial.print(" - ");
  Serial.print(tirePressure);
  Serial.print(" = ");
  Serial.print(finalPressure);
  Serial.println(" PSI");
}
float calcTirePressure() {
  // where
  // a = sensor reading
  // c = offset (103)
  // x = i dont know (232.0603804)
  // b = i dont know also (625)
  return (analogRead(pressureInputPin) - c) * (x / b);
}
void getTirePressure() {
  if (hasStartedGettingPressureSamples) {
    if (timeElapsed - lastSampleRead >= sampleTimeout) {
      lastSampleRead = timeElapsed;

      // subtract the last reading:
      total = total - pressureSamples[readIndex];

      // read from the sensor:
      pressureSamples[readIndex] = calcTirePressure();
      Serial.print(F("Reading tire pressure >> "));
      Serial.println(pressureSamples[readIndex]);

      // add the reading to the total:
      total = total + pressureSamples[readIndex];

      // advance to the next position in the array:
      readIndex++;

      // if we're at the end of the array...
      if (readIndex >= numOfPressureSamples) {
        strcpy(currentScreen, "displayPressure");
        hasStartedGettingPressureSamples = false;
        Serial.print(F("Initial pressure inside the tire >> "));
        Serial.println(tirePressure);
        readIndex = 0;
      }

      tirePressure = total / numOfPressureSamplesInFloat;
    }
  }
}
void clearPressureSamples() {
  for (int i = 0; i < numOfPressureSamples; i++) {
    pressureSamples[i] = 0;
  }
}
byte finalPressureDigitCount() {
  int finalPressureToInt = static_cast<int>(finalPressure);
  if (finalPressureToInt >= 100) {
    return 3;
  }
  if (finalPressureToInt < 100 && finalPressureToInt >= 10) {
    return 2;
  }
  if (finalPressureToInt < 10) {
    return 1;
  }
}
int getCarStandardPressure() {
  // sedan
  if (!strcmp(carType, "sedan")) {
    // loaded
    if (isLoaded) {
      if (isFront) {
        return sedanLoadedFrontPressure;
      } else {
        return sedanLoadedRearPressure;
      }

      // non-loaded
    } else {
      if (isFront) {
        return sedanNonloadedFrontPressure;
      } else {
        return sedanNonloadedRearPressure;
      }
    }
  }

  // suv
  if (!strcmp(carType, "suv")) {
    // loaded
    if (isLoaded) {
      if (isFront) {
        return suvLoadedFrontPressure;
      } else {
        return suvLoadedRearPressure;
      }

      // non-loaded
    } else {
      if (isFront) {
        return suvNonloadedFrontPressure;
      } else {
        return suvNonloadedRearPressure;
      }
    }
  }

  // pickup
  if (!strcmp(carType, "pickup")) {
    // loaded
    if (isLoaded) {
      if (isFront) {
        return pickupLoadedFrontPressure;
      } else {
        return pickupLoadedRearPressure;
      }

      // non-loaded
    } else {
      if (isFront) {
        return pickupNonloadedFrontPressure;
      } else {
        return pickupNonloadedRearPressure;
      }
    }
  }
}
int getBikeStandardPressure() {
  if (isBikeTireFront) {
    return bikeStandardFrontPressure;
  } else {
    return bikeStandardRearPressure;
  }
}
int getMotorStandardPressure() {
  if (isMotorTireFront) {
    return motorStandardFrontPressure;
  } else {
    return motorStandardRearPressure;
  }
}
//------------------------------ END PRESSURE -------------------------------------




//------------------------------ AIR INJECTION ------------------------------------

void injectAir() {
  if (hasInsertedCoin) {
    if (strcmp(currentScreen, "pressureStatus")) {
      strcpy(currentScreen, "pressureStatus");
      // inflator/compressor ON state
      digitalWrite(relayPin, LOW);
    }

    if (timeElapsed - lastPressureRead >= pressureReadTimeout) {
      lastPressureRead = timeElapsed;
      currentPressure = calcTirePressure();
      if (currentPressure >= finalPressure) {
        if (tirePressureSamples >= maxTirePressureSampleCount) {
          lastRestart = timeElapsed;
          digitalWrite(buzzerPin, LOW); // ON state
          digitalWrite(relayPin, HIGH); // OFF state
          strcpy(currentScreen, "done");
          hasInsertedCoin = false;
          restartRoutine = true;
          tirePressureSamples = 0;
          Serial.print(F("Done injecting air! Current tire pressure >> "));
          Serial.println(currentPressure);
        } else {
          Serial.println(F("Not yet."));
          tirePressureSamples++;
        }
      } else {
        displayCurrentPressure();
        Serial.print(F("Injecting air; Current pressure >> "));
        Serial.println(currentPressure);
      }
    }
  }

  // done injecting air
  // restart
  if (!hasInsertedCoin && restartRoutine) {
    if (timeElapsed - lastRestart >= restartTimeout) {
      strcpy(currentScreen, "chooseMode");
      restartRoutine = false;
      digitalWrite(buzzerPin, HIGH);
    }
  }
}

//------------------------------ END AIR INJECTION --------------------------------




//----------------------------- BACKGROUND SERVICES --------------------------------

// main
void runBackgroundServices() {
  notifyHasReachedCoinLimit();

  getMessage();
  parseCommand();
  sendReplyToCommand();

  getCurrentTime();
  sendIncome();
}

// reached coin limit
void notifyHasReachedCoinLimit() {
  if (strcmp(currentRoutine, "reachedCoinLimit")) {
    return;
  }

  if (hasReachedCoinLimit) {
    bool isSent = sendSms(reachedCoinLimitMessage);
    if (isSent) {
      if (replyCount >= maxReplyCount) {
        Serial.println(F("Next routine >> GET MESSAGE"));
        strcpy(currentRoutine, "getMessage");
        replyCount = 0;
        hasReachedCoinLimit = false;
      }
      Serial.println("Message sent");
      replyCount++;
    }
  } else {
    Serial.println(F("Next routine >> GET MESSAGE"));
    strcpy(currentRoutine, "getMessage");
  }
}

// Message parser
void sendReplyToCommand() {
  if (strcmp(currentRoutine, "sendReply")) {
    return;
  }

  bool replyHasBeenSent = sendSms(reply);
  if (replyHasBeenSent) {
    if (replyCount >= maxReplyCount) {
      strcpy(currentRoutine, "getCurrentTime");
      replyCount = 0;
      reply[0] = NULL;
      receivedCommand[0] = NULL;
    }

    Serial.println(F("Message sent!"));
    replyCount++;
  }
}
void getMessage() {
  if (strcmp(currentRoutine, "getMessage")) {
    return;
  }

  // send command
  if (timeElapsed - gsmStartedAt >= gsmTimeout) {
    gsmStartedAt = timeElapsed;
    if (!hasBeenSetToTextMode) {
      gsmSerial.print("AT+CMGF=1\r\n");
      hasBeenSetToTextMode = true;
    } else {
      hasBeenSetToTextMode = false;
      gsmSerial.print("AT+CMGR=1\r\n");
    }
  }

  // read response immediately
  readLine(8);

  if (isGsmResponseReady) {
    // search keywords in the message
    char *message = strstr(gsmResponse, "+CMGR: \"REC READ\"");
    // message contains keywords
    if (message != NULL) {
      Serial.println(F("Next routine >> PARSE COMMAND"));
      strcpy(currentRoutine, "parseCommand");
      strcpy(tempGsmResponse, gsmResponse);

      // immediately delete message
      gsmSerial.print("AT+CMGD=1,4\r\n");

      // reset retry count
      retryCount = 0;

      // message DOES NOT contain keywords
    } else {
      Serial.print(F("Retry count >> "));
      Serial.println(retryCount);

      if (retryCount >= maxRetryCount) {
        Serial.println(F("Next routine >> GET CURRENT TIME"));
        strcpy(currentRoutine, "getCurrentTime");
        retryCount = 0;
      }

      retryCount++;
    }

    isGsmResponseReady = false;
    gsmResponse[0] = NULL;
  }
}
void parseCommand() {
  if (strcmp(currentRoutine, "parseCommand")) {
    return;
  }

  Serial.print(F("Parsing command >> "));
  Serial.println(tempGsmResponse);

  // grab owner number from EEPROM
  char ownerNumber[14];
  EEPROM.get(ownerNumberAddress, ownerNumber);

  // check if command from owner
  char *isOwner = strstr(tempGsmResponse, ownerNumber);
  // not owner
  if (isOwner == NULL) {
    // set next routine
    Serial.println(F("Not the owner; Next routine >> GET CURRENT TIME"));
    strcpy(currentRoutine, "getCurrentTime");
    return;
  }

  // check command
  char *isCommandGet = strstr(tempGsmResponse, "GET");
  char *isCommandReset = strstr(tempGsmResponse, "RESET");
  // Get
  if (isCommandGet != NULL) {
    // get current income
    int currentIncome = getCurrentIncome();
    if (currentIncome == 0) {
      sprintf(reply, "There is currently no income.");
    } else {
      sprintf(reply, "Current income is %i pesos.", getCurrentIncome());
    }
  }

  // Reset
  if (isCommandReset != NULL) {
    byte numOfCoins = 0;
    EEPROM.get(numOfCoinsAddress, numOfCoins);
    Serial.print(F("Current number of coins to reset >> "));
    Serial.println(numOfCoins);

    // reset income
    EEPROM.update(numOfCoinsAddress, 0);
    // set message buffer
    strcpy(reply, "The income has been reset!");
    Serial.println(F("Command is RESET"));
  }

  // unknown
  if (!isCommandReset && !isCommandGet) {
    // set message buffer
    strcpy(reply, "Unknown command.");
    Serial.println(F("Command is UNKNOWN"));
  }

  Serial.println(F("Next routine >> SEND REPLY"));
  strcpy(currentRoutine, "sendReply");
}

// Midnight checker
void sendIncome() {
  if (strcmp(currentRoutine, "sendIncome")) {
    return;
  }

  if (strlen(reply) == 0) {
    sprintf(reply, "Midnight! Current income is %i pesos.", getCurrentIncome());
  }

  bool incomeHasBeenSent = sendSms(reply);
  if (incomeHasBeenSent) {
    if (replyCount >= maxReplyCount) {
      //      strcpy(currentRoutine, "getMessage");
      strcpy(currentRoutine, "reachedCoinLimit");
      replyCount = 0;
      reply[0] = NULL;
    }

    Serial.println(F("Message sent!"));
    replyCount++;
  }
}
void getCurrentTime() {
  if (strcmp(currentRoutine, "getCurrentTime")) {
    return;
  }

  if (timeElapsed - gsmStartedAt >= gsmTimeout && !isGettingTime) {
    gsmStartedAt = timeElapsed;
    gsmSerial.print("AT+CCLK?\r\n");
    isGettingTime = true;
  }

  readLine(1);

  isGettingTime = false;
  if (isGsmResponseReady) {
    const char delimiters[] = ",+";
    char *response;
    char *timePtr;

    strcpy(tempGsmResponse, gsmResponse);
    Serial.print(F("Response >> "));
    Serial.println(tempGsmResponse);

    response = strstr(tempGsmResponse, "+CCLK:");
    Serial.println(F("Getting current time"));
    if (response != NULL) {
      if (strtok(gsmResponse, delimiters) != NULL) {
        // hh:mm:ss
        timePtr = strtok(NULL, delimiters);

        Serial.print(F("Current time >> "));
        Serial.println(timePtr);

        if (timePtr != NULL) {
          bool isMidnight = checkIfMidnight(timePtr);
          if (isMidnight) {
            Serial.println(F("Midnight >> YES"));
            Serial.println(F("Next routine >> SEND INCOME"));
            strcpy(currentRoutine, "sendIncome");
            return;
          } else {
            Serial.println(F("Midnight >> NO"));
          }
        }
      }

      Serial.println(F("Next routine >> REACHED COIN LIMIT"));
      //      strcpy(currentRoutine, "getMessage");
      strcpy(currentRoutine, "reachedCoinLimit");
    }

    isGsmResponseReady = false;
    gsmResponse[0] = NULL;
  }
}
bool checkIfMidnight(char *currentTime) {
  const char delimiter[] = ":";
  char *_hour;
  char *_minute;
  char *_second;
  bool isMidnight = false;

  _hour = strtok(currentTime, delimiter);
  if (_hour != NULL) {
    _minute = strtok(NULL, delimiter);
    if (_minute != NULL && atoi(_minute) % 2 != 0) {
      isMidnight = true;
      //      _second = strtok(NULL, delimiter);
      //      if (atoi(_second) >= 10 && atoi(_second) <= 15) {
      //        isMidnight = true;
      //      }
    }
  }

  return isMidnight;
}


//---------------------------- END BACKGROUND SERVICES -----------------------------




//------------------------------ HELPER FUNCTIONS  --------------------------------

bool sendSms(char *message) {
  if (!hasStartedSendingSms) {
    strcpy(currentCommand, "txtMode");
    strcpy(prevCommand, "txtMode");

    gsmSerial.println("AT+CMGF=1");
    Serial.println(F("Sending SMS..."));
    Serial.println("AT+CMGF=1");
    startedAt = timeElapsed;
    hasStartedSendingSms = true;
  }

  if (timeElapsed - startedAt >= commandInterval && hasStartedSendingSms) {
    startedAt = timeElapsed;
    if (!strcmp(prevCommand, "txtMode")) {
      char setContactCommand[32];
      char ownerNumber[16];
      EEPROM.get(ownerNumberAddress, ownerNumber);
      sprintf(setContactCommand, "AT+CMGS=\"%s\"\r\n", ownerNumber);
      strcpy(currentCommand, "contact");
      Serial.println("Setting contact");
      gsmSerial.print(setContactCommand);
    }
    if (!strcmp(currentCommand, "contact") && strcmp(prevCommand, "txtMode")) {
      Serial.print("Setting reply >> ");
      Serial.println(message);
      strcpy(currentCommand, "message");
      gsmSerial.println(message);
    }
    if (!strcmp(currentCommand, "message") && strcmp(prevCommand, "contact")) {
      Serial.println("Writing end character");
      strcpy(currentCommand, "end");
      gsmSerial.println((char)26);
    }
    if (!strcmp(currentCommand, "end") && strcmp(prevCommand, "message")) {
      currentCommand[0] = NULL;
      hasStartedSendingSms = false;
      return true;
    }

    strcpy(prevCommand, currentCommand);
  }

  return false;
}
void clearEEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}
void setReachedCoinLimitMessage() {
  sprintf(reachedCoinLimitMessage, "The device has reached %i-coin limit.", coinCountLimit);
  Serial.print(F("Reached coin limit message set!"));
}
int getCurrentIncome() {
  byte numOfCoins = 0;
  EEPROM.get(numOfCoinsAddress, numOfCoins);

  return numOfCoins * 5;
}
void readLine(byte maxLineCount) {
  static byte index = 0;
  static byte lineCount = 0;
  char endMarker = '\n';
  char rc;

  while (gsmSerial.available() > 0 && !isGsmResponseReady) {
    rc = gsmSerial.read();

    // receive char while not end marker
    if (rc != endMarker) {
      gsmResponse[index] = rc;
      index++;

      // buffer overrun guard
      if (index >= numChars) {
        index = numChars - 1;
      }
      return;
    }

    // if end marker
    // check line count
    if (lineCount < maxLineCount) {
      gsmResponse[index] = ' ';
      index++;
      lineCount++;

      // response is ready
    } else {
      gsmResponse[index] = NULL;
      index = 0;
      lineCount = 0;
      isGsmResponseReady = true;
    }
  }
}


//---------------------------- END HELPER FUNCTIONS -------------------------------
