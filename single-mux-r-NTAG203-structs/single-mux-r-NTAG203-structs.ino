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
*   - Readers connected to TCA9548A channels 1 and 2
*   - Version checking of RFID2 reader bypassed due to WS1850S/MFRC522 differences
* ----------------------------------------------
*/

#include <Wire.h>
#include <MFRC522v2.h>
#include <MFRC522DriverI2C.h>
#include <MFRC522Debug.h>

enum LEDState {
  LED_OFF,
  LED_GREEN,
  LED_RED
};

// ----- I2C Addresses -----
const uint8_t TCA9548A_ADDR = 0x70;
const uint8_t RFID2_WS1850S_ADDR = 0x28;

// ----- TCA9548A Channels -----
const uint8_t NEGATIVE_TERMINAL_CHANNEL = 0;  // Channel for - terminal reader on battery
const uint8_t POSITIVE_TERMINAL_CHANNEL = 1;  // Channel for + terminal reader on battery

// ----- LED Test Pins -----
const uint8_t GREEN_LED_PIN = 5;
const uint8_t RED_LED_PIN = 6;

// RFID driver and reader instance
MFRC522DriverI2C driver{ RFID2_WS1850S_ADDR, Wire };
MFRC522 reader{ driver };

const uint8_t TAG_START_READ_PAGE = 4;

// ----- DATA STRUCTURES -----
struct JumperCableTagData {
  char type[4];      // either "POS" or "NEG"
  uint8_t id;        // 1, 2, 3, or 4 (for the 4 cable ends)
  uint8_t checksum;  // simple validation
};

struct TerminalReader {
  const char* name;            // human-friendly name lol
  uint8_t channel;             // reader's MUX channel (neg is 0, pos is 1)
  bool isReaderOK;             // boolean for reader health/status, set by reader initializer function
  bool isTagPresent;           // is a tag present
  bool isCorrectPolarity;      // if tag is present, does data (polarity) match terminal?
  JumperCableTagData tagData;  // tag data
};

struct Battery {
  uint8_t muxAddress;
  uint8_t id;
  TerminalReader positive;
  TerminalReader negative;
  bool hasValidConfiguration() const {
    // both terminals must have tags present, check that here
    if (!positive.isTagPresent || !negative.isTagPresent) return false;

    // both cards must have correct polarity for their terminals
    if (!positive.isCorrectPolarity || !negative.isCorrectPolarity) return false;

    // cable ID's must form a valid pair (clamp 1 and 3 or clamp 2 and 4)
    uint8_t posID = positive.tagData.id;
    uint8_t negID = negative.tagData.id;

    return (posID == 1 && negID == 3) || (posID == 2 && negID == 4);
  }
};

// ----- GLOBAL BATTERY INSTANCE -----
Battery battery = {
  TCA9548A_ADDR,
  1,  // id
  {
    // positive terminal
    "Positive Terminal",        // name
    POSITIVE_TERMINAL_CHANNEL,  // channel
    false,                      // isReaderOK
    false,                      // isTagPresent
    false,                      // isCorrectPolarity
    {}                          // tagData
  },
  { // negative terminal
    "Negative Terminal",
    NEGATIVE_TERMINAL_CHANNEL,
    false,
    false,
    false,
    {} },
};

// ========== MUX SET CHANNEL ==========
void TCA9548A_setChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ========== MUX DISABLE ALL CHANNELS ==========
void TCA9548A_disableChannels() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ========== CHECKSUM CALC ==========
uint8_t calculateChecksum(const uint8_t* data, uint8_t length) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum ^= data[i];  // XOR checksum
  }
  return sum;
}

// ========== RESET READER'S TAG DATA ==========
void clearTagData(TerminalReader& terminal) {
  terminal.isTagPresent = false;
  terminal.isCorrectPolarity = false;
  // wipe whole struct clean in one call if no tag detected:
  memset(&terminal.tagData, 0, sizeof(terminal.tagData));
}

// ========== MAIN RFID INITIALIZER ===========
void initializeReader(TerminalReader& terminal) {
  Serial.print("Initializing ");
  Serial.print(terminal.name);
  Serial.print(" (Channel ");
  Serial.print(terminal.channel);
  Serial.print(")...");

  TCA9548A_setChannel(terminal.channel);

  // I2C communication test
  Wire.beginTransmission(RFID2_WS1850S_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("FAILED - I2C Communication ERROR");
    terminal.isReaderOK = false;
    return;
  }

  // initialize reader
  reader.PCD_Init();
  delay(50);

  Serial.println(" SUCCESS");
  terminal.isReaderOK = true;
  TCA9548A_disableChannels();
  return;
}

// ========== CHECK FOR TAG ON READER ==========
bool checkForTag(TerminalReader& terminal) {
  TCA9548A_setChannel(terminal.channel);
  delay(5);
  terminal.isTagPresent = (reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial());
  return terminal.isTagPresent;
}

// ========== TAG READING FUNCTION ==========
// (only invoked if tag is detected in the first place, see checkForTag() function)
void readTagData(TerminalReader& terminal) {
  if (!terminal.isReaderOK || !terminal.isTagPresent) return;

  Serial.print(terminal.name);
  Serial.println(": Reading tag data...");

  TCA9548A_setChannel(terminal.channel);
  delay(5);

  // create byte array (buffer) to store data from tag
  byte buffer[18];
  byte bufferSize = sizeof(buffer);

  // NOTE: view MFRC522 library src code on github or my read/write repo (https://github.com/iwonder77/rw-NTAG203-rfid-tag)
  // for more info on this MIFARE_Read() function
  if (reader.MIFARE_Read(TAG_START_READ_PAGE, buffer, &bufferSize) != MFRC522::StatusCode::STATUS_OK) {
    Serial.print(terminal.name);
    Serial.println(": Failed to read card data");
    clearTagData(terminal);
    return;
  }

  // cast raw data from buffer into data variable of type JumperCableTagData struct
  JumperCableTagData data;
  memcpy(&data, buffer, sizeof(JumperCableTagData));

  // Validate checksum
  uint8_t expectedChecksum = calculateChecksum((uint8_t*)&data, sizeof(data) - 1);
  if (expectedChecksum != data.checksum) {
    Serial.print(terminal.name);
    Serial.println(": Checksum error");
    clearTagData(terminal);
    return;
  }

  terminal.tagData = data;

  // Check if polarity matches terminal
  bool isTagPos = (strncmp(data.type, "POS", 3) == 0);
  bool isTerminalPos = (terminal.channel == POSITIVE_TERMINAL_CHANNEL);
  // Will return true for neg tag to neg term, and pos tag to pos term, false otherwise
  terminal.isCorrectPolarity = (isTagPos == isTerminalPos);

  // Print status
  Serial.print(terminal.name);
  Serial.print(": Detected ");
  Serial.print(data.type);
  Serial.print(" cable #");
  Serial.print(data.id);

  if (!terminal.isCorrectPolarity) {
    Serial.print(" [WRONG POLARITY!]");
  }
  Serial.println();

  return;
}

// ===== LED Control =====
void updateLEDs() {
  static LEDState lastState = LED_OFF;
  LEDState currentState;

  // count how many terminals have tags present
  uint8_t presentCount = (battery.positive.isTagPresent ? 1 : 0) + (battery.negative.isTagPresent ? 1 : 0);

  if (presentCount < 2) {
    currentState = LED_OFF;
  } else if (battery.hasValidConfiguration()) {
    currentState = LED_GREEN;
  } else {
    currentState = LED_RED;
  }

  if (currentState != lastState) {
    switch (currentState) {
      case LED_OFF:
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, LOW);
        Serial.println("LEDs OFF - waiting for both tags");
        break;
      case LED_GREEN:
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
        Serial.println("✅ Correct configuration - Green ON");
        break;
      case LED_RED:
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.println("❌ Incorrect configuration - Red ON");
        break;
    }
    // Print detailed status
    Serial.println("--- Configuration Status ---");
    Serial.print("Positive Terminal: ");
    if (!battery.positive.isTagPresent) {
      Serial.println("No card");
    } else {
      Serial.print(battery.positive.tagData.type);
      Serial.print(" #");
      Serial.print(battery.positive.tagData.id);
      Serial.println(battery.positive.isCorrectPolarity ? " ✓" : " ✗ Wrong polarity");
    }
    Serial.print("Negative Terminal: ");
    if (!battery.negative.isTagPresent) {
      Serial.println("No card");
    } else {
      Serial.print(battery.negative.tagData.type);
      Serial.print(" #");
      Serial.print(battery.negative.tagData.id);
      Serial.println(battery.negative.isCorrectPolarity ? " ✓" : " ✗ Wrong polarity");
    }
    Serial.println("----------------------------");
    lastState = currentState;
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== Battery RFID System v2.0 ===");

  // Initialize LEDs
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  // Initialize I2C
  Wire.begin();

  // Initialize multiplexer
  TCA9548A_disableChannels();
  delay(50);

  // Initialize both readers
  initializeReader(battery.positive);
  delay(50);
  initializeReader(battery.negative);

  if (!battery.positive.isReaderOK && !battery.negative.isReaderOK) {
    Serial.println("ERROR: No readers responding! Check wiring.");
    while (1) {
      // Blink red LED to indicate error
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
      delay(500);
    }
  }
  if (!battery.positive.isReaderOK) {
    Serial.println("Positive Terminal reader not responding");
  }
  if (!battery.negative.isReaderOK) {
    Serial.println("Negative Terminal reader not responding");
  }

  Serial.println("\n=== System Ready ===");
  Serial.println("Place jumper cable tags on terminals to test");

  TCA9548A_disableChannels();
}

// ========== MAIN LOOP ==========
void loop() {
  Serial.println("\n--- Starting scan cycle ---");

  // Check positive terminal
  Serial.println("Checking POSITIVE terminal...");
  if (checkForTag(battery.positive)) {
    Serial.println("Card detected!");
    readTagData(battery.positive);
    TCA9548A_disableChannels();
  }

  delay(50);  // Small delay between channels

  // Check negative terminal
  Serial.println("Checking NEGATIVE terminal...");
  if (checkForTag(battery.negative)) {
    Serial.println("Card detected!");
    readTagData(battery.negative);
    TCA9548A_disableChannels();
  }

  // Update LEDs
  updateLEDs();

  // Disable all channels between cycles
  TCA9548A_disableChannels();

  Serial.println("--- End scan cycle ---\n");
  delay(3000);  // Longer delay to make output readable
}
