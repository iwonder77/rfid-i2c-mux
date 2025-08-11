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
*     3V3/GND pins
*
*     * Notes:
*   - RFID2 boards have internal pull-up resistors for SDA/SCL lines
*   - SDA/SCL lines operate at 3.3V logic level
*   - Readers connected to TCA9548A channels 1 and 2
*   - Version checking bypassed due to WS1850S/MFRC522 differences
* ----------------------------------------------
*/

#include <Wire.h>
#include <MFRC522v2.h>
#include <MFRC522DriverI2C.h>
#include <MFRC522Debug.h>

// ----- I2C Addresses -----
const uint8_t TCA9548A_ADDR = 0x70;
const uint8_t RFID2_WS1850S_ADDR = 0x28;

// ----- TCA9548A Channels -----
const uint8_t READER1_CHANNEL = 1;
const uint8_t READER2_CHANNEL = 2;

// single driver and reader instance
MFRC522DriverI2C driver{ RFID2_WS1850S_ADDR, Wire };
MFRC522 reader{ driver };

void TCA9548A_setChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void TCA9548A_disable() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// Test if reader can detect cards (bypasses version checking)
bool testReaderFunctionality(uint8_t channel, const char* readerName) {
  Serial.print("Testing ");
  Serial.print(readerName);
  Serial.print(" functionality...");

  TCA9548A_setChannel(channel);
  delay(50);

  // Initialize without checking version
  reader.PCD_Init();
  delay(100);

  // Test basic I2C communication
  Wire.beginTransmission(RFID2_WS1850S_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(" I2C FAILED");
    return false;
  }

  Serial.println(" I2C OK, RFID functionality assumed working");
  return true;
}

bool initializeReader(uint8_t channel, const char* readerName) {
  Serial.print("Initializing ");
  Serial.print(readerName);
  Serial.print(" (Channel ");
  Serial.print(channel);
  Serial.print(")...");

  if (!testReaderFunctionality(channel, readerName)) {
    Serial.println(" FAILED");
    return false;
  }

  TCA9548A_setChannel(channel);
  reader.PCD_Init();
  delay(100);

  Serial.println(" SUCCESS");
  return true;
}

bool checkForCard(uint8_t channel) {
  TCA9548A_setChannel(channel);
  delay(5);
  return (reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial());
}

void processCard(uint8_t channel, const char* readerName) {
  TCA9548A_setChannel(channel);

  Serial.print("Card detected on ");
  Serial.print(readerName);
  Serial.print(" (Channel ");
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

  Serial.println("---");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== TCA9548A RFID Reader - Working Version ===");
  Serial.println("Version checking bypassed for WS1850S compatibility\n");

  Wire.begin();

  // Disable all channels initially
  TCA9548A_disable();
  delay(100);

  // Initialize both readers (no version checking)
  bool reader1_ok = initializeReader(READER1_CHANNEL, "Reader 1");
  bool reader2_ok = initializeReader(READER2_CHANNEL, "Reader 2");

  Serial.print("\nInitialization complete: Reader1=");
  Serial.print(reader1_ok ? "OK" : "FAILED");
  Serial.print(", Reader2=");
  Serial.println(reader2_ok ? "OK" : "FAILED");

  if (!reader1_ok && !reader2_ok) {
    Serial.println("ERROR: No readers responding! Check wiring.");
    while (1) delay(1000);
  }

  // Show some basic info (without version details that don't work)
  if (reader1_ok) {
    Serial.println("\nReader 1 ready for card detection");
  }

  if (reader2_ok) {
    Serial.println("Reader 2 ready for card detection");
  }

  Serial.println("\n=== Place cards on readers to test ===");
  Serial.println("Remove and replace cards to see detection\n");

  TCA9548A_disable();
}

void loop() {
  // Check Reader 1
  if (checkForCard(READER1_CHANNEL)) {
    processCard(READER1_CHANNEL, "Reader 1");
    delay(1000);  // Prevent spam detection
  }

  // Check Reader 2
  if (checkForCard(READER2_CHANNEL)) {
    processCard(READER2_CHANNEL, "Reader 2");
    delay(1000);  // Prevent spam detection
  }

  delay(100);
}
