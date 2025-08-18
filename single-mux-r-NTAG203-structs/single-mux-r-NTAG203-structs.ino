/* 
* ----------------------------------------------
* PROJECT NAME: TCA9548A_I2C_Multiplexer 
* Description: using Adafruit's TCA9548A I2C multiplexer to gather RFID information from two M5Stack WS1850S RFID2 readers 
*
* Author: Isai Sanchez Date: 8-11-25 
* Board(s) Used: Arduino Nano 
* Libraries: 
*   - Wire.h (I2C communication library): https://docs.arduino.cc/language-reference/en/functions/communication/wire/ 
*   - MFRC522v2.h (Main RFID library): https://github.com/OSSLibraries/Arduino_MFRC522v2 
*
* Notes: 
*   - The RFID2 readers used have a WS1850S chip rather than the MFRC522, so there are subtle differences that the library
*     doesn't play nice with, however reading the datasheet for the MFRC522 and the src code for the library seems to help and work out alright
*       sidenote: coudln't for the life of me figure out how to download the datasheet for these RFID readers from M5Stack, 
*       definitely will go with other options if we use RFID again
*   - Found through trial and error that the RFID2 boards have internal pull-up resistors for the SDA/SCL lines. So these were 
*     connected straight to the TCA9548A multiplexer's output channels (SD1/SC1 and SD2/SC2) without the use of an external pull up resistor
*   - Also found out that the SDA/SCL lines for the RFID2 readers are at 3.3V logic level, so the multiplexer was powered with Arduino's
*     3V3/GND pins
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

enum TagState {
  TAG_ABSENT,
  TAG_DETECTED,
  TAG_PRESENT,
  TAG_REMOVED
};

// ===================== CONSTANTS ====================
// ----- I2C Addresses -----
const uint8_t TCA9548A_ADDR = 0x70;
const uint8_t RFID2_WS1850S_ADDR = 0x28;

// ----- TCA9548A Channels -----
const uint8_t NEGATIVE_TERMINAL_CHANNEL = 0;
const uint8_t POSITIVE_TERMINAL_CHANNEL = 1;

// ----- LED Test Pins -----
const uint8_t GREEN_LED_PIN = 5;
const uint8_t RED_LED_PIN = 6;

// ----- Timing Constants -----
const unsigned long TAG_POLL_INTERVAL = 250;     // how often to check for tags (ms)
const unsigned long TAG_ABSENCE_TIMEOUT = 1000;  // time before considering tag removed (ms)
const unsigned long TAG_DEBOUNCE_TIME = 100;     // debounce time for tag detection (ms)
const uint8_t TAG_PRESENCE_THRESHOLD = 3;        // consecutive reading fails before marking absent

const uint8_t TAG_START_READ_PAGE = 4;
// ====================================================

// ===================== HARDWARE INSTANCES ====================
MFRC522DriverI2C driver{ RFID2_WS1850S_ADDR, Wire };  // // RFID driver and reader instance
MFRC522 reader{ driver };
// =============================================================

// ===================== DATA STRUCTURES ====================
struct JumperCableTagData {
  char type[4];      // either "POS" or "NEG"
  uint8_t id;        // 1, 2, 3, or 4 (for the 4 cable ends)
  uint8_t checksum;  // simple validation
};

struct TerminalReader {
  const char* name;
  uint8_t channel;
  bool isReaderOK;
  TagState tagState;            // New: State machine for tag
  unsigned long lastSeenTime;   // New: Track when tag was last detected
  unsigned long firstSeenTime;  // New: Track when tag was first detected
  uint8_t consecutiveFails;     // New: Count failed detection attempts
  bool isCorrectPolarity;
  JumperCableTagData tagData;
  byte lastUID[10];    // New: Store UID to detect tag changes
  byte lastUIDLength;  // New: UID length
};

struct Battery {
  uint8_t muxAddress;
  uint8_t id;
  TerminalReader positive;
  TerminalReader negative;
  bool hasValidConfiguration() const {
    // Both terminals must have tags in PRESENT state
    if (positive.tagState != TAG_PRESENT || negative.tagState != TAG_PRESENT) return false;

    // Both cards must have correct polarity
    if (!positive.isCorrectPolarity || !negative.isCorrectPolarity) return false;

    // Cable IDs must form valid pair
    uint8_t posID = positive.tagData.id;
    uint8_t negID = negative.tagData.id;

    return (posID == 1 && negID == 3) || (posID == 2 && negID == 4);
  }
};

// ----- GLOBAL BATTERY INSTANCE -----
Battery battery = {
  TCA9548A_ADDR,
  1,
  { "Positive Terminal",
    POSITIVE_TERMINAL_CHANNEL,
    false,
    TAG_ABSENT,
    0,
    0,
    0,
    false,
    {},
    {},
    0 },
  { "Negative Terminal",
    NEGATIVE_TERMINAL_CHANNEL,
    false,
    TAG_ABSENT,
    0,
    0,
    0,
    false,
    {},
    {},
    0 },
};
// =========================================================


// ========== MUX FUNCTIONS ==========
void TCA9548A_setChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void TCA9548A_disableChannels() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ========== UTILITY FUNCTIONS ==========
uint8_t calculateChecksum(const uint8_t* data, uint8_t length) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum ^= data[i];
  }
  return sum;
}

bool compareUID(byte* uid1, byte* uid2, byte length) {
  for (byte i = 0; i < length; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

void clearTagData(TerminalReader& terminal) {
  terminal.isCorrectPolarity = false;
  memset(&terminal.tagData, 0, sizeof(terminal.tagData));
  terminal.lastUIDLength = 0;
  memset(terminal.lastUID, 0, sizeof(terminal.lastUID));
}

// ========== RFID FUNCTIONS ==========
void initializeReader(TerminalReader& terminal) {
  Serial.print("Initializing ");
  Serial.print(terminal.name);
  Serial.print(" (Channel ");
  Serial.print(terminal.channel);
  Serial.print(")...");

  TCA9548A_setChannel(terminal.channel);

  Wire.beginTransmission(RFID2_WS1850S_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("FAILED - I2C Communication ERROR");
    terminal.isReaderOK = false;
    return;
  }

  reader.PCD_Init();
  delay(50);

  Serial.println(" SUCCESS");
  terminal.isReaderOK = true;
}

// ========== TAG DETECTION WITH STATE MACHINE ==========
void updateTagState(TerminalReader& terminal) {
  if (!terminal.isReaderOK) return;

  TCA9548A_setChannel(terminal.channel);
  delay(5);

  unsigned long currentTime = millis();
  bool tagDetected = false;

  // Try to detect tag without halting it
  if (reader.PICC_IsNewCardPresent()) {
    if (reader.PICC_ReadCardSerial()) {
      tagDetected = true;

      // Check if this is the same tag or a different one
      bool isSameTag = (terminal.lastUIDLength == reader.uid.size) && compareUID(terminal.lastUID, reader.uid.uidByte, reader.uid.size);

      // Update UID
      memcpy(terminal.lastUID, reader.uid.uidByte, reader.uid.size);
      terminal.lastUIDLength = reader.uid.size;

      // Update timing
      terminal.lastSeenTime = currentTime;
      terminal.consecutiveFails = 0;

      // State transitions
      switch (terminal.tagState) {
        case TAG_ABSENT:
          terminal.tagState = TAG_DETECTED;
          terminal.firstSeenTime = currentTime;
          Serial.print(terminal.name);
          Serial.println(": New tag detected!");
          break;

        case TAG_DETECTED:
          // Check if enough time has passed for debouncing
          if (currentTime - terminal.firstSeenTime > TAG_DEBOUNCE_TIME) {
            terminal.tagState = TAG_PRESENT;
            Serial.print(terminal.name);
            Serial.println(": Tag confirmed present");
            // Read the tag data when transitioning to PRESENT
            readTagData(terminal);
          }
          break;

        case TAG_PRESENT:
          if (!isSameTag) {
            // Different tag detected
            terminal.tagState = TAG_DETECTED;
            terminal.firstSeenTime = currentTime;
            clearTagData(terminal);
            Serial.print(terminal.name);
            Serial.println(": Different tag detected!");
          }
          // Same tag still present - no action needed
          break;

        case TAG_REMOVED:
          terminal.tagState = TAG_DETECTED;
          terminal.firstSeenTime = currentTime;
          Serial.print(terminal.name);
          Serial.println(": Tag returned!");
          break;
      }

      // Don't halt the tag - let it stay active
      // We're NOT calling PICC_HaltA() here
    }
  }

  // Handle absence detection
  if (!tagDetected && terminal.tagState != TAG_ABSENT) {
    terminal.consecutiveFails++;

    // Use different logic based on current state
    if (terminal.tagState == TAG_DETECTED) {
      // Quick timeout for tags that were just detected
      if (terminal.consecutiveFails >= 2) {
        terminal.tagState = TAG_ABSENT;
        clearTagData(terminal);
        Serial.print(terminal.name);
        Serial.println(": Tag detection failed");
      }
    } else if (terminal.tagState == TAG_PRESENT) {
      // More lenient for established tags
      if (terminal.consecutiveFails >= TAG_PRESENCE_THRESHOLD || (currentTime - terminal.lastSeenTime > TAG_ABSENCE_TIMEOUT)) {
        terminal.tagState = TAG_REMOVED;
        Serial.print(terminal.name);
        Serial.println(": Tag removed!");
      }
    } else if (terminal.tagState == TAG_REMOVED) {
      // Confirm removal
      if (currentTime - terminal.lastSeenTime > TAG_ABSENCE_TIMEOUT * 2) {
        terminal.tagState = TAG_ABSENT;
        clearTagData(terminal);
        Serial.print(terminal.name);
        Serial.println(": Tag removal confirmed");
      }
    }
  }
}

// ========== READ TAG DATA (only when needed) ==========
void readTagData(TerminalReader& terminal) {
  if (!terminal.isReaderOK || terminal.tagState != TAG_PRESENT) return;

  TCA9548A_setChannel(terminal.channel);
  delay(5);

  Serial.print(terminal.name);
  Serial.println(": Reading tag data...");

  byte buffer[18];
  byte bufferSize = sizeof(buffer);

  if (reader.MIFARE_Read(TAG_START_READ_PAGE, buffer, &bufferSize) != MFRC522::StatusCode::STATUS_OK) {
    Serial.print(terminal.name);
    Serial.println(": Failed to read card data");
    // Don't clear tag data - we know tag is present, just couldn't read it
    return;
  }

  JumperCableTagData data;
  memcpy(&data, buffer, sizeof(JumperCableTagData));

  uint8_t expectedChecksum = calculateChecksum((uint8_t*)&data, sizeof(data) - 1);
  if (expectedChecksum != data.checksum) {
    Serial.print(terminal.name);
    Serial.println(": Checksum error");
    return;
  }

  terminal.tagData = data;

  bool isTagPos = (strncmp(data.type, "POS", 3) == 0);
  bool isTerminalPos = (terminal.channel == POSITIVE_TERMINAL_CHANNEL);
  terminal.isCorrectPolarity = (isTagPos == isTerminalPos);

  Serial.print(terminal.name);
  Serial.print(": Read ");
  Serial.print(data.type);
  Serial.print(" cable #");
  Serial.print(data.id);

  if (!terminal.isCorrectPolarity) {
    Serial.print(" [WRONG POLARITY!]");
  }
  Serial.println();

  // Still don't halt - let tag remain active for continuous detection
}

// ========== POLLING FUNCTION ==========
void pollRFIDReaders() {
  static unsigned long lastPollTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastPollTime < TAG_POLL_INTERVAL) {
    return;  // Not time to poll yet
  }

  lastPollTime = currentTime;

  // Update both terminals
  updateTagState(battery.positive);
  updateTagState(battery.negative);
}

// ========== LED CONTROL ==========
void updateLEDs() {
  static LEDState lastState = LED_OFF;
  LEDState currentState;

  // Count terminals with present tags
  uint8_t presentCount = 0;
  if (battery.positive.tagState == TAG_PRESENT) presentCount++;
  if (battery.negative.tagState == TAG_PRESENT) presentCount++;

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

    printConfigurationStatus();
    lastState = currentState;
  }
}

void printConfigurationStatus() {
  Serial.println("--- Configuration Status ---");

  Serial.print("Positive Terminal: ");
  printTerminalStatus(battery.positive);

  Serial.print("Negative Terminal: ");
  printTerminalStatus(battery.negative);

  Serial.println("----------------------------");
}

void printTerminalStatus(const TerminalReader& terminal) {
  switch (terminal.tagState) {
    case TAG_ABSENT:
      Serial.println("No card");
      break;
    case TAG_DETECTED:
      Serial.println("Detecting...");
      break;
    case TAG_PRESENT:
      Serial.print(terminal.tagData.type);
      Serial.print(" #");
      Serial.print(terminal.tagData.id);
      Serial.println(terminal.isCorrectPolarity ? " ✓" : " ✗ Wrong polarity");
      break;
    case TAG_REMOVED:
      Serial.println("Card removed (confirming...)");
      break;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== Battery RFID System v3.0 ===");
  Serial.println("Features: Persistent tag detection with state machine");

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  Wire.begin();

  TCA9548A_disableChannels();
  delay(50);

  initializeReader(battery.positive);
  delay(50);
  initializeReader(battery.negative);

  if (!battery.positive.isReaderOK && !battery.negative.isReaderOK) {
    Serial.println("ERROR: No readers responding! Check wiring.");
    while (1) {
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
      delay(500);
    }
  }

  if (!battery.positive.isReaderOK) {
    Serial.println("Warning: Positive Terminal reader not responding");
  }
  if (!battery.negative.isReaderOK) {
    Serial.println("Warning: Negative Terminal reader not responding");
  }

  Serial.println("\n=== System Ready ===");
  Serial.println("Place jumper cable tags on terminals to test");

  TCA9548A_disableChannels();
}

// ========== MAIN LOOP ==========
void loop() {
  // Poll RFID readers at controlled intervals
  pollRFIDReaders();

  // Update LED status
  updateLEDs();

  // Small delay for system stability
  delay(10);
}
