/*  CANBed RP2040 V1.1 CAN Logger
  Firmware: V0.1

  Suggested Git commit:
  Add passive CAN logger with SD binary capture and persistent bitrate configuration

  Purpose:
  - Listen to all Classical CAN 2.0 traffic without transmitting.
  - Store received frames as fixed-size binary records on an SD card.
  - Detect the CAN bitrate at startup when automatic detection is enabled.
  - Fall back to the persistent bitrate if detection is inconclusive.
  - Provide serial diagnostics and persistent configuration commands.

  Hardware:
  CANBed RP2040 V1.1
  MCP2515 CAN controller
  SD card on shared SPI bus

  SPI:
    SCK  = GPIO 2
    MOSI = GPIO 3
    MISO = GPIO 4
    CAN CS = GPIO 9
    SD CS  = GPIO 8
    CAN INT = GPIO 11

  Notes:
  - The Longan MCP2515 library is used for CAN access.
  - This first implementation deliberately uses polling rather than an
    interrupt service routine. The CAN INT pin is retained for diagnostics
    and for a later capture-path optimization.
  - The MCP2515 is placed in listen-only mode before capture.
  - Bitrate detection is passive. No CAN frames are transmitted.
*/

#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <mcp_canbus.h>

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------

constexpr uint8_t CAN_CS_PIN  = 9;
constexpr uint8_t SD_CS_PIN   = 8;
constexpr uint8_t CAN_INT_PIN = 11;

constexpr uint32_t SERIAL_BAUD = 115200;

// Default configuration.
constexpr uint32_t DEFAULT_BITRATE = 500000;

// Detection starts with the two most likely rates.
// Additional rates are tried if the first two do not produce convincing data.
constexpr uint32_t DETECTION_RATES[] = {
  250000,
  500000,
  125000,
  1000000,
  100000,
  200000,
  800000,
  666000,
  83300
};

constexpr size_t DETECTION_RATE_COUNT =
  sizeof(DETECTION_RATES) / sizeof(DETECTION_RATES[0]);

// Five seconds is the maximum total detection period.
constexpr uint32_t DETECTION_WINDOW_MS = 5000;

// A candidate must produce this many valid frames before it is accepted.
constexpr uint32_t DETECTION_MIN_FRAMES = 5;

// Runtime health monitoring.
constexpr uint32_t HEALTH_CHECK_INTERVAL_MS = 1000;

// A sustained run of CAN controller errors triggers a re-scan.
// This is deliberately conservative for V0.1.
constexpr uint32_t HEALTH_ERROR_THRESHOLD = 10;

// RAM ring buffer.
// 20-byte disk record × 600 entries = 12,000 bytes.
constexpr size_t LOG_BUFFER_RECORDS = 600;

// SD writes are batched. This keeps SD latency away from the CAN polling path.
constexpr size_t SD_BATCH_RECORDS = 32;

// -----------------------------------------------------------------------------
// Persistent configuration
// -----------------------------------------------------------------------------

constexpr uint32_t CONFIG_MAGIC = 0x43414E4CUL;  // "CANL"
constexpr uint16_t CONFIG_VERSION = 1;

struct PersistentConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t bitrate;
  uint8_t autoDetect;
  uint8_t reserved2[3];
  uint32_t checksum;
};

constexpr int EEPROM_SIZE = 64;
PersistentConfig config;

// -----------------------------------------------------------------------------
// Binary file format
// -----------------------------------------------------------------------------

constexpr uint32_t LOG_MAGIC = 0x43414E4CUL; // "CANL"
constexpr uint16_t LOG_VERSION = 1;
constexpr uint16_t LOG_RECORD_SIZE = 20;

struct CanLogRecord {
  uint32_t timestamp_us;
  uint32_t can_id;
  uint8_t flags;
  uint8_t dlc;
  uint8_t data[8];
  uint8_t reserved[2];
};

static_assert(sizeof(CanLogRecord) == LOG_RECORD_SIZE,
              "CAN log record must remain 20 bytes.");

struct LogFileHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;
  uint32_t recordSize;
  uint32_t bitrate;
  uint32_t startMillis;
  uint8_t reserved[12];
};

static_assert(sizeof(LogFileHeader) == 32,
              "Log header must remain 32 bytes.");

// -----------------------------------------------------------------------------
// CAN object and logger state
// -----------------------------------------------------------------------------

MCP_CAN CAN(CAN_CS_PIN);

File logFile;

CanLogRecord logBuffer[LOG_BUFFER_RECORDS];

volatile size_t bufferHead = 0;
volatile size_t bufferTail = 0;
volatile size_t bufferCount = 0;
size_t peakBufferCount = 0;

uint64_t framesReceived = 0;
uint64_t framesWritten = 0;
uint64_t framesDropped = 0;
uint64_t sdWriteErrors = 0;
uint64_t sdWriteOperations = 0;

uint32_t maxSdWriteUs = 0;
uint32_t lastSdWriteUs = 0;

uint32_t activeBitrate = DEFAULT_BITRATE;

uint32_t lastHealthCheckMs = 0;
uint32_t healthErrorCount = 0;

bool loggingActive = false;
bool sdReady = false;
bool canReady = false;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------

void printStatus();
void printHelp();
bool initializeSD();
bool openNextLogFile();
bool initializeCAN(uint32_t bitrate, bool listenOnly);
bool setListenOnlyMode();
bool detectBitrate(uint32_t &detectedBitrate);
bool validateCanCandidate(uint32_t bitrate, uint32_t durationMs,
                          uint32_t &validFrames);
bool enqueueCanFrame(uint32_t canId, uint8_t flags, uint8_t dlc,
                     const uint8_t *data);
bool flushLogBuffer();
bool writeLogHeader();
void serviceSerial();
void processSerialCommand(const char *command);
void saveConfig();
void loadConfig();
uint32_t configChecksum(const PersistentConfig &cfg);
bool validBitrate(uint32_t bitrate);
void handleRuntimeHealth();
void triggerBitrateRescan();
void printBitrate(uint32_t bitrate);
bool isSupportedLonganBitrate(uint32_t bitrate);
int bitrateToLibraryConstant(uint32_t bitrate);

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Do not wait forever for a USB serial connection. The logger is intended
  // to operate headlessly when powered from a vehicle or power source.
  delay(300);

  Serial.println();
  Serial.println(F("CAN LOGGER V0.1"));
  Serial.println(F("----------------"));
  Serial.println(F("CANBed RP2040 V1.1"));
  Serial.println(F("Mode: passive CAN capture"));
  Serial.println();

  pinMode(CAN_INT_PIN, INPUT);

  // The CAN controller and SD card share SPI. Both chip-selects must be
  // inactive before either peripheral is initialized.
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  SPI.begin();

  loadConfig();

  Serial.print(F("Stored bitrate: "));
  printBitrate(config.bitrate);
  Serial.print(F("Auto detection: "));
  Serial.println(config.autoDetect ? F("ENABLED") : F("DISABLED"));

  if (!initializeSD()) {
    Serial.println(F("FATAL: SD initialization failed."));
    return;
  }

  if (config.autoDetect) {
    uint32_t detectedBitrate = 0;

    Serial.println(F("Starting passive bitrate detection..."));

    if (detectBitrate(detectedBitrate)) {
      activeBitrate = detectedBitrate;

      Serial.print(F("Detected bitrate: "));
      printBitrate(activeBitrate);
    } else {
      activeBitrate = config.bitrate;

      Serial.println(F("Detection inconclusive."));
      Serial.print(F("Using stored bitrate: "));
      printBitrate(activeBitrate);
    }
  } else {
    activeBitrate = config.bitrate;
  }

  if (!initializeCAN(activeBitrate, true)) {
    Serial.println(F("FATAL: CAN initialization failed."));
    return;
  }

  if (!openNextLogFile()) {
    Serial.println(F("FATAL: Could not create log file."));
    return;
  }

  loggingActive = true;
  lastHealthCheckMs = millis();

  Serial.println();
  Serial.println(F("Logging started."));
  Serial.print(F("Active bitrate: "));
  printBitrate(activeBitrate);
  Serial.println(F("CAN mode: LISTEN ONLY"));
  Serial.println(F("SD logging: ACTIVE"));
  Serial.println();
  printHelp();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop() {
  serviceSerial();

  if (!loggingActive || !canReady) {
    delay(10);
    return;
  }

  // Drain all currently available CAN frames before doing other work.
  // This minimizes the time that the MCP2515 receive buffers remain full.
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t canId = 0;
    uint8_t len = 0;
    uint8_t data[8] = {0};

    byte result = CAN.readMsgBufID(&canId, &len, data);

    if (result != CAN_OK) {
      healthErrorCount++;
      continue;
    }

    uint8_t flags = 0;

    if (CAN.isExtendedFrame()) {
      flags |= 0x01;
    }

    if (CAN.isRemoteRequest()) {
      flags |= 0x02;
    }

    if (!enqueueCanFrame(canId, flags, len, data)) {
      framesDropped++;
    } else {
      framesReceived++;
    }
  }

  // Keep SD operations away from individual CAN frame reception.
  if (bufferCount >= SD_BATCH_RECORDS) {
    flushLogBuffer();
  }

  if (millis() - lastHealthCheckMs >= HEALTH_CHECK_INTERVAL_MS) {
    handleRuntimeHealth();
    lastHealthCheckMs = millis();
  }
}

// -----------------------------------------------------------------------------
// SD initialization
// -----------------------------------------------------------------------------

bool initializeSD() {
  Serial.print(F("Initializing SD card... "));

  // Explicitly de-select the CAN controller while the SD card is accessed.
  digitalWrite(CAN_CS_PIN, HIGH);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FAILED"));
    sdReady = false;
    return false;
  }

  Serial.println(F("OK"));
  sdReady = true;
  return true;
}

// -----------------------------------------------------------------------------
// Log file creation
// -----------------------------------------------------------------------------

bool openNextLogFile() {
  char filename[20];

  for (uint32_t index = 1; index <= 99999; index++) {
    snprintf(filename, sizeof(filename), "/CAN_%05lu.BIN",
             static_cast<unsigned long>(index));

    if (!SD.exists(filename)) {
      Serial.print(F("Creating log file: "));
      Serial.println(filename);

      digitalWrite(CAN_CS_PIN, HIGH);

      logFile = SD.open(filename, FILE_WRITE);

      if (!logFile) {
        Serial.println(F("Failed to open log file."));
        return false;
      }

      if (!writeLogHeader()) {
        logFile.close();
        return false;
      }

      return true;
    }
  }

  Serial.println(F("No available log filename."));
  return false;
}

bool writeLogHeader() {
  LogFileHeader header = {};

  header.magic = LOG_MAGIC;
  header.version = LOG_VERSION;
  header.headerSize = sizeof(LogFileHeader);
  header.recordSize = LOG_RECORD_SIZE;
  header.bitrate = activeBitrate;
  header.startMillis = millis();

  size_t written = logFile.write(
    reinterpret_cast<const uint8_t *>(&header),
    sizeof(header)
  );

  if (written != sizeof(header)) {
    Serial.println(F("Failed to write log header."));
    sdWriteErrors++;
    return false;
  }

  logFile.flush();
  return true;
}

// -----------------------------------------------------------------------------
// CAN initialization
// -----------------------------------------------------------------------------

bool initializeCAN(uint32_t bitrate, bool listenOnly) {
  int libraryRate = bitrateToLibraryConstant(bitrate);

  if (libraryRate < 0) {
    Serial.print(F("Unsupported CAN bitrate: "));
    printBitrate(bitrate);
    return false;
  }

  // De-select SD before accessing the MCP2515.
  digitalWrite(SD_CS_PIN, HIGH);

  Serial.print(F("Initializing CAN at "));
  printBitrate(bitrate);

  byte result = CAN.begin(static_cast<byte>(libraryRate));

  if (result != CAN_OK) {
    Serial.println(F("CAN initialization FAILED"));
    canReady = false;
    return false;
  }

  if (listenOnly) {
    if (!setListenOnlyMode()) {
      Serial.println(F("Failed to enter CAN listen-only mode."));
      canReady = false;
      return false;
    }
  }

  Serial.println(F("CAN initialization OK"));
  canReady = true;
  return true;
}

// -----------------------------------------------------------------------------
// MCP2515 listen-only mode
// -----------------------------------------------------------------------------

bool setListenOnlyMode() {
  /*
    Longan's library exposes the MCP2515 internals needed to change the
    controller mode, but the public API does not provide a listen-only
    wrapper in all released versions.

    The MCP2515 CANCTRL register is:
      address 0x0F
      mode bits 7:5

    Listen-only mode is 0b011xxxxx = 0x60.

    We perform the mode change directly over the shared SPI bus here so that
    V0.1 does not depend on an unreleased or fork-specific library API.
  */

  constexpr uint8_t MCP_CANCTRL = 0x0F;
  constexpr uint8_t MCP_CANSTAT = 0x0E;
  constexpr uint8_t MCP_WRITE    = 0x02;
  constexpr uint8_t MODE_MASK    = 0xE0;
  constexpr uint8_t MODE_LISTENONLY = 0x60;

  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(CAN_CS_PIN, LOW);

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  SPI.transfer(MCP_WRITE);
  SPI.transfer(MCP_CANCTRL);
  SPI.transfer(MODE_LISTENONLY);

  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();

  delay(2);

  // Read CANSTAT to confirm the requested mode.
  digitalWrite(CAN_CS_PIN, LOW);

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  SPI.transfer(0x03);  // READ
  SPI.transfer(MCP_CANSTAT);
  uint8_t status = SPI.transfer(0x00);

  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();

  return (status & MODE_MASK) == MODE_LISTENONLY;
}

// -----------------------------------------------------------------------------
// Bitrate detection
// -----------------------------------------------------------------------------

bool detectBitrate(uint32_t &detectedBitrate) {
  uint32_t detectionStart = millis();

  for (size_t i = 0; i < DETECTION_RATE_COUNT; i++) {
    uint32_t elapsed = millis() - detectionStart;

    if (elapsed >= DETECTION_WINDOW_MS) {
      break;
    }

    uint32_t remaining = DETECTION_WINDOW_MS - elapsed;

    // Divide remaining time between the remaining candidates. A successful
    // candidate terminates detection immediately.
    size_t remainingCandidates = DETECTION_RATE_COUNT - i;
    uint32_t candidateWindow = remaining / remainingCandidates;

    // Never give a candidate less than 100 ms unless the overall window is
    // almost exhausted.
    if (candidateWindow < 100 && remaining >= 100) {
      candidateWindow = 100;
    }

    uint32_t validFrames = 0;

    Serial.print(F("  Trying "));
    printBitrate(DETECTION_RATES[i]);

    if (validateCanCandidate(DETECTION_RATES[i],
                             candidateWindow,
                             validFrames)) {

      Serial.print(F("    valid frames: "));
      Serial.println(validFrames);

      if (validFrames >= DETECTION_MIN_FRAMES) {
        detectedBitrate = DETECTION_RATES[i];
        Serial.println(F("    ACCEPTED"));
        return true;
      }
    } else {
      Serial.println(F("    no valid traffic"));
    }
  }

  return false;
}

bool validateCanCandidate(uint32_t bitrate,
                          uint32_t durationMs,
                          uint32_t &validFrames) {
  validFrames = 0;

  // Initialize the controller at the candidate bitrate and immediately put
  // it into listen-only mode. No CAN frame can be transmitted by this code.
  if (!initializeCAN(bitrate, true)) {
    return false;
  }

  uint32_t start = millis();

  while (millis() - start < durationMs) {
    serviceSerial();

    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      uint32_t canId = 0;
      uint8_t len = 0;
      uint8_t data[8] = {0};

      if (CAN.readMsgBufID(&canId, &len, data) == CAN_OK) {
        validFrames++;
      }
    }
  }

  return validFrames > 0;
}

// -----------------------------------------------------------------------------
// CAN receive buffer
// -----------------------------------------------------------------------------

bool enqueueCanFrame(uint32_t canId,
                     uint8_t flags,
                     uint8_t dlc,
                     const uint8_t *data) {
  if (bufferCount >= LOG_BUFFER_RECORDS) {
    return false;
  }

  CanLogRecord &record = logBuffer[bufferHead];

  record.timestamp_us = micros();
  record.can_id = canId;
  record.flags = flags;
  record.dlc = dlc;

  memset(record.data, 0, sizeof(record.data));

  if (dlc > 0) {
    memcpy(record.data, data, min<uint8_t>(dlc, 8));
  }

  record.reserved[0] = 0;
  record.reserved[1] = 0;

  bufferHead++;
  if (bufferHead >= LOG_BUFFER_RECORDS) {
    bufferHead = 0;
  }

  bufferCount++;

  if (bufferCount > peakBufferCount) {
    peakBufferCount = bufferCount;
  }

  return true;
}

// -----------------------------------------------------------------------------
// SD buffer flush
// -----------------------------------------------------------------------------

bool flushLogBuffer() {
  if (!logFile || bufferCount == 0) {
    return true;
  }

  // The CAN controller must not be selected while the SD card is active.
  digitalWrite(CAN_CS_PIN, HIGH);

  size_t recordsToWrite = min(bufferCount, SD_BATCH_RECORDS);

  uint32_t startUs = micros();

  for (size_t i = 0; i < recordsToWrite; i++) {
    size_t written = logFile.write(
      reinterpret_cast<const uint8_t *>(&logBuffer[bufferTail]),
      sizeof(CanLogRecord)
    );

    if (written != sizeof(CanLogRecord)) {
      sdWriteErrors++;
      return false;
    }

    bufferTail++;
    if (bufferTail >= LOG_BUFFER_RECORDS) {
      bufferTail = 0;
    }

    bufferCount--;
    framesWritten++;
  }

  logFile.flush();

  uint32_t elapsedUs = micros() - startUs;

  lastSdWriteUs = elapsedUs;
  if (elapsedUs > maxSdWriteUs) {
    maxSdWriteUs = elapsedUs;
  }

  sdWriteOperations++;

  return true;
}

// -----------------------------------------------------------------------------
// Runtime health
// -----------------------------------------------------------------------------

void handleRuntimeHealth() {
  static uint64_t previousReceived = 0;
  static uint64_t previousWritten = 0;
  static uint64_t previousDropped = 0;

  uint64_t receivedDelta = framesReceived - previousReceived;
  uint64_t writtenDelta = framesWritten - previousWritten;
  uint64_t droppedDelta = framesDropped - previousDropped;

  previousReceived = framesReceived;
  previousWritten = framesWritten;
  previousDropped = framesDropped;

  // The serial output is intentionally low-rate. It does not print every CAN
  // frame and therefore does not become part of the capture bottleneck.
  Serial.print(F("[health] rx="));
  Serial.print(static_cast<unsigned long>(receivedDelta));
  Serial.print(F("/s written="));
  Serial.print(static_cast<unsigned long>(writtenDelta));
  Serial.print(F("/s dropped="));
  Serial.print(static_cast<unsigned long>(droppedDelta));
  Serial.print(F(" buffer="));
  Serial.print(bufferCount);
  Serial.print(F("/"));
  Serial.print(LOG_BUFFER_RECORDS);
  Serial.print(F(" peak="));
  Serial.print(peakBufferCount);
  Serial.print(F(" maxSDus="));
  Serial.print(maxSdWriteUs);
  Serial.print(F(" sdErr="));
  Serial.println(static_cast<unsigned long>(sdWriteErrors));

  // A dropped software record is a capture-capacity problem, not evidence
  // that the CAN bitrate is wrong.
  if (droppedDelta > 0) {
    Serial.println(F("[warning] CAN capture buffer overflow"));
  }

  // V0.1 does not automatically rescan solely because no frames arrive.
  // An inactive CAN bus is a valid condition.
  if (healthErrorCount >= HEALTH_ERROR_THRESHOLD) {
    healthErrorCount = 0;
    triggerBitrateRescan();
  }
}

// -----------------------------------------------------------------------------
// Runtime bitrate rescan
// -----------------------------------------------------------------------------

void triggerBitrateRescan() {
  if (!loggingActive) {
    return;
  }

  Serial.println();
  Serial.println(F("[warning] CAN health degraded"));
  Serial.println(F("Starting 5-second bitrate rescan..."));

  loggingActive = false;

  // Preserve records already captured before changing the MCP2515 setup.
  flushLogBuffer();

  uint32_t detectedBitrate = 0;

  if (detectBitrate(detectedBitrate)) {
    activeBitrate = detectedBitrate;

    Serial.print(F("Rescan selected "));
    printBitrate(activeBitrate);

    if (!initializeCAN(activeBitrate, true)) {
      Serial.println(F("Rescan CAN initialization failed."));
      return;
    }
  } else {
    activeBitrate = config.bitrate;

    Serial.print(F("Rescan inconclusive. Returning to stored bitrate "));
    printBitrate(activeBitrate);

    if (!initializeCAN(activeBitrate, true)) {
      Serial.println(F("Fallback CAN initialization failed."));
      return;
    }
  }

  loggingActive = true;
  Serial.println(F("CAN capture resumed."));
}

// -----------------------------------------------------------------------------
// Serial command processing
// -----------------------------------------------------------------------------

void serviceSerial() {
  static char commandBuffer[64];
  static size_t commandLength = 0;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processSerialCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

void processSerialCommand(const char *command) {
  if (strcmp(command, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(command, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "bitrate") == 0) {
    Serial.print(F("Stored bitrate: "));
    printBitrate(config.bitrate);
    Serial.print(F("Active bitrate: "));
    printBitrate(activeBitrate);
    return;
  }

  if (strcmp(command, "autodetect on") == 0) {
    config.autoDetect = 1;
    saveConfig();
    Serial.println(F("Automatic bitrate detection ENABLED and saved."));
    return;
  }

  if (strcmp(command, "autodetect off") == 0) {
    config.autoDetect = 0;
    saveConfig();
    Serial.println(F("Automatic bitrate detection DISABLED and saved."));
    return;
  }

  if (strcmp(command, "bitrate auto") == 0) {
    config.autoDetect = 1;
    saveConfig();
    Serial.println(F("Automatic bitrate detection ENABLED and saved."));
    return;
  }

  if (strncmp(command, "bitrate ", 8) == 0) {
    uint32_t requested = strtoul(command + 8, nullptr, 10);

    if (!isSupportedLonganBitrate(requested)) {
      Serial.println(F("Unsupported bitrate."));
      Serial.println(F("Supported: 50000, 83300, 100000, 125000, 200000,"));
      Serial.println(F("          250000, 500000, 666000, 800000, 1000000"));
      return;
    }

    config.bitrate = requested;
    saveConfig();

    Serial.print(F("Stored bitrate set to "));
    printBitrate(config.bitrate);

    Serial.println(F("Restart logger or use a power cycle to apply it."));
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
}

// -----------------------------------------------------------------------------
// Persistent configuration
// -----------------------------------------------------------------------------

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);

  bool valid =
    config.magic == CONFIG_MAGIC &&
    config.version == CONFIG_VERSION &&
    validBitrate(config.bitrate) &&
    config.checksum == configChecksum(config);

  if (!valid) {
    config.magic = CONFIG_MAGIC;
    config.version = CONFIG_VERSION;
    config.reserved = 0;
    config.bitrate = DEFAULT_BITRATE;
    config.autoDetect = 1;
    memset(config.reserved2, 0, sizeof(config.reserved2));
    config.checksum = configChecksum(config);

    EEPROM.put(0, config);
    EEPROM.commit();

    Serial.println(F("No valid saved configuration. Using defaults."));
  }
}

void saveConfig() {
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;
  config.checksum = configChecksum(config);

  EEPROM.put(0, config);
  EEPROM.commit();
}

uint32_t configChecksum(const PersistentConfig &cfg) {
  const uint8_t *bytes =
    reinterpret_cast<const uint8_t *>(&cfg);

  uint32_t checksum = 2166136261UL;

  // Exclude the checksum field itself.
  size_t count = offsetof(PersistentConfig, checksum);

  for (size_t i = 0; i < count; i++) {
    checksum ^= bytes[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

// -----------------------------------------------------------------------------
// Bitrate helpers
// -----------------------------------------------------------------------------

bool validBitrate(uint32_t bitrate) {
  return isSupportedLonganBitrate(bitrate);
}

bool isSupportedLonganBitrate(uint32_t bitrate) {
  switch (bitrate) {
    case 50000:
    case 83300:
    case 100000:
    case 125000:
    case 200000:
    case 250000:
    case 500000:
    case 666000:
    case 800000:
    case 1000000:
      return true;

    default:
      return false;
  }
}

int bitrateToLibraryConstant(uint32_t bitrate) {
  switch (bitrate) {
    case 50000:   return CAN_50KBPS;
    case 83300:   return CAN_83K3BPS;
    case 100000:  return CAN_100KBPS;
    case 125000:  return CAN_125KBPS;
    case 200000:  return CAN_200KBPS;
    case 250000:  return CAN_250KBPS;
    case 500000:  return CAN_500KBPS;
    case 666000:  return CAN_666KBPS;
    case 800000:
      // The Longan library does not provide a portable 800 kbit/s constant
      // in all releases. It is therefore not used until a matching MCP2515
      // timing configuration is added.
      return -1;
    case 1000000: return CAN_1000KBPS;

    default:
      return -1;
  }
}

void printBitrate(uint32_t bitrate) {
  Serial.print(bitrate);
  Serial.println(F(" bit/s"));
}

// -----------------------------------------------------------------------------
// Serial diagnostics
// -----------------------------------------------------------------------------

void printStatus() {
  Serial.println();
  Serial.println(F("CAN LOGGER STATUS"));
  Serial.println(F("-----------------"));

  Serial.print(F("Stored bitrate:       "));
  printBitrate(config.bitrate);

  Serial.print(F("Active bitrate:       "));
  printBitrate(activeBitrate);

  Serial.print(F("Auto detection:       "));
  Serial.println(config.autoDetect ? F("ON") : F("OFF"));

  Serial.print(F("CAN ready:             "));
  Serial.println(canReady ? F("YES") : F("NO"));

  Serial.print(F("SD ready:              "));
  Serial.println(sdReady ? F("YES") : F("NO"));

  Serial.print(F("Logging:               "));
  Serial.println(loggingActive ? F("ACTIVE") : F("STOPPED"));

  Serial.print(F("Frames received:      "));
  Serial.println(static_cast<unsigned long>(framesReceived));

  Serial.print(F("Frames written:       "));
  Serial.println(static_cast<unsigned long>(framesWritten));

  Serial.print(F("Frames dropped:       "));
  Serial.println(static_cast<unsigned long>(framesDropped));

  Serial.print(F("Buffer:               "));
  Serial.print(bufferCount);
  Serial.print(F("/"));
  Serial.println(LOG_BUFFER_RECORDS);

  Serial.print(F("Peak buffer:          "));
  Serial.println(peakBufferCount);

  Serial.print(F("SD write operations:  "));
  Serial.println(static_cast<unsigned long>(sdWriteOperations));

  Serial.print(F("Last SD write us:     "));
  Serial.println(lastSdWriteUs);

  Serial.print(F("Max SD write us:      "));
  Serial.println(maxSdWriteUs);

  Serial.print(F("SD write errors:      "));
  Serial.println(static_cast<unsigned long>(sdWriteErrors));

  if (logFile) {
    Serial.print(F("Log file size:        "));
    Serial.println(static_cast<unsigned long>(logFile.size()));
  }

  Serial.println();
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  bitrate"));
  Serial.println(F("  bitrate 125000"));
  Serial.println(F("  bitrate 250000"));
  Serial.println(F("  bitrate 500000"));
  Serial.println(F("  bitrate 1000000"));
  Serial.println(F("  bitrate auto"));
  Serial.println(F("  autodetect on"));
  Serial.println(F("  autodetect off"));
}
'''

path = Path("/mnt/data/CANLogger_V01_passive_SD_20261024_1257.ino")
path.write_text(code)
print(path)
