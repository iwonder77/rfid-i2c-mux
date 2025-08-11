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
* ----------------------------------------------
*/

#include <Wire.h>
#include <MFRC522v2.h>
#include <MFRC522DriverI2C.h>  // tells the library ot use I2C
#include <MFRC522Debug.h>      // extra functions for detailed output

// ----- I2C Addresses -----
const uint8_t TCA9548A_ADDR = 0x70;       // multiplexer
const uint8_t RFID2_WS1850S_ADDR = 0x28;  // RFID2 unit

// ----- TCA9548A Channels -----
const uint8_t READER1_CHANNEL = 1;
const uint8_t READER2_CHANNEL = 2;

// single driver and reader instance - we'll switch channels manually

MFRC522DriverI2C driver{ RFID2_WS1850S_ADDR, Wire };  // creates I2C driver
MFRC522 reader{ driver };                             // creates RFID object for the readers (MFRC522 instance).

void TCA9548A_setChannel(uint8_t channel) {
  if (channel > 7) return;  // safety check (only 8 channels available 0-7)

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void TCA9548A_disable() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

bool initializeReader(uint8_t channel, const char* readerName) {
  TCA9548A_setChannel(channel);
  delay(10);

  reader.PCD_Init();

  if (reader.PCD_GetVersion() == MFRC522::PCD_Version::Version_Unknown) {
    Serial.print("ERROR: ");
    Serial.print(readerName);
    Serial.println(" not responding!");
    return false;
  }

  Serial.print(readerName);
  Serial.print(" initialized successfully (Channel ");
  Serial.print(channel);
  Serial.println(")");
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
    if (reader.uid.uidByte[i] < 0x10) Serial.print("0");  // Leading zero for single hex digits
    Serial.print(reader.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Optional: Print detailed card information
  // MFRC522Debug::PICC_DumpToSerial(reader, Serial, &(reader.uid));

  // Halt PICC and stop encryption on PCD
  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
}



void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C with default SDA and SCL pins

  Serial.println("=== TCA9548A RFID Reader Demo ===");

  Wire.begin();  // Initialize I2C

  // Disable all channels initially
  TCA9548A_disable();
  delay(100);

  // Initialize both readers
  bool reader1_ok = initializeReader(READER1_CHANNEL, "Reader 1");
  bool reader2_ok = initializeReader(READER2_CHANNEL, "Reader 2");

  if (!reader1_ok && !reader2_ok) {
    Serial.println("ERROR: No readers responding!");
    while (1);  // Halt execution
  }

  // Show version info for working readers
  if (reader1_ok) {
    TCA9548A_setChannel(READER1_CHANNEL);
    Serial.println("\n--- Reader 1 Details ---");
    MFRC522Debug::PCD_DumpVersionToSerial(reader, Serial);
  }

  if (reader2_ok) {
    TCA9548A_setChannel(READER2_CHANNEL);
    Serial.println("\n--- Reader 2 Details ---");
    MFRC522Debug::PCD_DumpVersionToSerial(reader, Serial);
  }

  Serial.println("\nPlace cards on readers...\n");
  TCA9548A_disable();


  MFRC522Debug::PCD_DumpVersionToSerial(reader, Serial);  // Show details of PCD - MFRC522 Card Reader details.
}


void loop() {
  if (checkForCard(READER1_CHANNEL)) {
    processCard(READER1_CHANNEL, "Reader 1");
  }
  if (checkForCard(READER2_CHANNEL)) {
    processCard(READER2_CHANNEL, "Reader 1");
  }

  delay(100);
}
