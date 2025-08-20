/* 
* ----------------------------------------------
* PROJECT NAME: TCA9548A_I2C_Multiplexer
* Description: using Adafruit's TCA9548A I2C multiplexer to gather RFID information from two M5Stack WS1850S RFID2 readers
* 
* Author: Isai Sanchez
* Date: 8-11-25
* Board Used: Arduino Nano
* Libraries:
*   - Wire.h (I2C communication library): https://docs.arduino.cc/language-reference/en/functions/communication/wire/
*   - MFRC522v2.h (Main RFID library): https://github.com/OSSLibraries/Arduino_MFRC522v2
*   - 
* Notes:
*   - Found through trial and error that the RFID2 boards have internal pull-up resistors for the SDA/SCL lines. So these were 
*     connected straight to the TCA9548A multiplexer's output channels (SD1/SC1 and SD2/SC2) without the use of an external pull up resistor
*   - Also found out that the SDA/SCL lines for the RFID2 readers are at 3.3V logic level, so the multiplexer was powered with Arduino's
*     3V3/GND pins to avoid logic level issues
*   - Readers connected to TCA9548A channels 1 and 2 and powered by external 5V supply line
*   - Readers contain the WS1850S chip, which is supposedly an upgrade to the MFRC522 chip, so some methods from the MFRC522 library, such as the
*     version checking methods, may not work as intended. 
* ----------------------------------------------
*/

#include <Wire.h>
#include <MFRC522v2.h>
#include <MFRC522DriverI2C.h>
#include <MFRC522Debug.h>

// ----- I2C Addresses -----
const uint8_t TCA9548A_MUX1_ADDR = 0x70;
const uint8_t TCA9548A_MUX2_ADDR = 0x71;
const uint8_t TCA9548A_MUX3_ADDR = 0x72;
const uint8_t RFID2_WS1850S_ADDR = 0x28;

// ----- TCA9548A Channels -----
const uint8_t NUM_BATTERIES = 3;
const uint8_t NEGATIVE_TERMINAL_CHANNEL = 0;  // Channel for - terminal on battery
const uint8_t POSITIVE_TERMINAL_CHANNEL = 1;  // Channel for + terminal on battery

// ----- BATTERY CONFIGURATION STRUCTURE -----
struct Battery {
  uint8_t muxAddress;
  const char* name;
  bool positiveReaderOK;
  bool negativeReaderOK;
};

// initialize battery configurations
Battery batteries[NUM_BATTERIES] = {
  { TCA9548A_MUX1_ADDR, "Battery 1", false, false },
  { TCA9548A_MUX2_ADDR, "Battery 2", false, false },
  { TCA9548A_MUX3_ADDR, "Battery 3", false, false },
};

// RFID driver and reader instance
MFRC522DriverI2C driver{ RFID2_WS1850S_ADDR, Wire };
MFRC522 reader{ driver };

// ---------- MUX SET CHANNEL ----------
void TCA9548A_setChannel(uint8_t muxAddress, uint8_t channel) {
  // disable all channels first before setting the next channel
  TCA9548A_disableAll();
  delay(5);
  // safety check (channels on TCA9548A are limited to 0-7)
  if (channel > 7) return;
  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channel);
  byte result = Wire.endTransmission();

  if (result != 0) {
    Serial.print("ERROR: MUX communication failed for address 0x");
    Serial.print(muxAddress, HEX);
    Serial.print(", result: ");
    Serial.println(result);
  }
}

// ---------- MUX DISABLE ALL CHANNELS ----------
void TCA9548A_disableAll() {
  // disable all channels on both multiplexers
  Wire.beginTransmission(TCA9548A_MUX1_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(TCA9548A_MUX2_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(TCA9548A_MUX3_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ---------- CARD DETECTION ----------
bool checkForCard(uint8_t muxAddress, uint8_t channel) {
  TCA9548A_setChannel(muxAddress, channel);
  delay(5);
  return (reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial());
}

// ---------- PROCESS CARD INFORMATION ----------
void processCard(uint8_t muxAddress, uint8_t channel, const char* location) {
  TCA9548A_setChannel(muxAddress, channel);

  Serial.print("Card detected on ");
  Serial.print(location);
  Serial.print(" (MUX 0x");
  Serial.print(muxAddress, HEX);
  Serial.print(", Channel ");
  Serial.print(channel);
  Serial.println(")");

  // Print UID
  Serial.print("UID: ");
  for (byte i = 0; i < reader.uid.size; i++) {
    if (reader.uid.uidByte[i] < 0x10) Serial.print("0");
    Serial.print(reader.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // WARNING: card type is a little sus
  // MFRC522::PICC_Type piccType = reader.PICC_GetType(reader.uid.sak);
  // Serial.print("Card type: ");
  // Serial.println(reader.PICC_GetType(piccType));

  // Optional: Print detailed card information
  // MFRC522Debug::PICC_DumpToSerial(reader, Serial, &(reader.uid));

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();

  Serial.println("----------");
}

// ---------- RFID READER TEST ----------
bool testReaderI2CCommunication(uint8_t muxAddress, uint8_t channel, const char* readerName) {
  Serial.print("Testing ");
  Serial.print(readerName);
  Serial.print(" functionality on MUX Ox");
  Serial.print(muxAddress, HEX);
  Serial.print(":");

  TCA9548A_setChannel(muxAddress, channel);
  delay(50);

  // initialize reader
  reader.PCD_Init();
  delay(100);

  // test basic I2C communication
  Wire.beginTransmission(RFID2_WS1850S_ADDR);
  byte result = Wire.endTransmission();

  if (result != 0) {
    Serial.print(readerName);
    Serial.println(" - I2C Communication FAILED");
    return false;
  }

  Serial.print(readerName);
  Serial.println(" - I2C OK");
  return true;
}

// ---------- RFID READER INITIALIZATION ----------
bool initializeReader(uint8_t muxAddress, uint8_t channel, const char* readerName) {
  if (!testReaderI2CCommunication(muxAddress, channel, readerName)) {
    Serial.println("INITIALIZATION FAILED");
    return false;
  }

  Serial.print(readerName);
  Serial.println(" - Initialization SUCCESS");
  return true;
}

// ---------- INITIALIZE OUR SYSTEM ----------
void initializeSystem() {
  Serial.println("=== Dual Battery RFID System Initialization ===");

  Wire.begin();
  TCA9548A_disableAll();
  delay(100);

  // test MUX communication first
  Serial.println("\nTesting MUX communication...");
  for (int i = 0; i < NUM_BATTERIES; i++) {
    Wire.beginTransmission(batteries[i].muxAddress);
    byte result = Wire.endTransmission();

    Serial.print(batteries[i].name);
    Serial.print(" MUX (0x");
    Serial.print(batteries[i].muxAddress, HEX);
    Serial.print("): ");
    Serial.println(result == 0 ? "OK" : "FAILED");
  }

  // then initialize all readers
  Serial.println("\nInitializing RFID readers...");
  for (int i = 0; i < NUM_BATTERIES; i++) {
    Serial.print("\n--- ");
    Serial.print(batteries[i].name);
    Serial.println(" ---");

    // Create descriptive names for terminals
    char posTerminalName[50];
    char negTerminalName[50];
    sprintf(posTerminalName, "%s Positive Terminal", batteries[i].name);
    sprintf(negTerminalName, "%s Negative Terminal", batteries[i].name);

    // Initialize positive terminal reader
    batteries[i].positiveReaderOK = initializeReader(
      batteries[i].muxAddress,
      POSITIVE_TERMINAL_CHANNEL,
      posTerminalName);

    // Initialize negative terminal reader
    batteries[i].negativeReaderOK = initializeReader(
      batteries[i].muxAddress,
      NEGATIVE_TERMINAL_CHANNEL,
      negTerminalName);
  }

  Serial.println("\n === Initialization Summary ===");
  bool anyReaderWorking = false;

  for (int i = 0; i < NUM_BATTERIES; i++) {
    Serial.print(batteries[i].name);
    Serial.print(": Positive=");
    Serial.print(batteries[i].positiveReaderOK ? "OK" : "FAILED");
    Serial.print(", Negative=");
    Serial.println(batteries[i].negativeReaderOK ? "OK" : "FAILED");

    if (batteries[i].positiveReaderOK || batteries[i].negativeReaderOK) {
      anyReaderWorking = true;
    }
  }
  if (!anyReaderWorking) {
    Serial.println("\nERROR: No readers responding! Check wiring and addresses.");
    while (1) delay(1000);
  }

  Serial.println("\n=== System Ready - Place RFID tags on terminals ===\n");
  TCA9548A_disableAll();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  initializeSystem();
}

void loop() {
  // Check all batteries and terminals
  for (int i = 0; i < NUM_BATTERIES; i++) {
    char posTerminalName[50];
    char negTerminalName[50];
    sprintf(posTerminalName, "%s Positive Terminal", batteries[i].name);
    sprintf(negTerminalName, "%s Negative Terminal", batteries[i].name);

    // Check posit ive terminal (only if reader is working)
    if (batteries[i].positiveReaderOK) {
      if (checkForCard(batteries[i].muxAddress, POSITIVE_TERMINAL_CHANNEL)) {
        processCard(batteries[i].muxAddress, POSITIVE_TERMINAL_CHANNEL, posTerminalName);
        delay(1000);  // Prevent spam
      }
    }

    // Check negative terminal (only if reader is working)
    if (batteries[i].negativeReaderOK) {
      if (checkForCard(batteries[i].muxAddress, NEGATIVE_TERMINAL_CHANNEL)) {
        processCard(batteries[i].muxAddress, NEGATIVE_TERMINAL_CHANNEL, negTerminalName);
        delay(1000);  // Prevent spam
      }
    }
  }

  delay(100);
}
