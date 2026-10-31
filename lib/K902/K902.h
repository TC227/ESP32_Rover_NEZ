#pragma once

#include <Arduino.h>

// Latest solution decoded from the K902's NMEA output. Values not reported yet stay at -1.
struct K902Fix {
  uint8_t quality = 0;         // GGA: 0 none, 1 single, 2 DGPS/SBAS, 4 RTK fixed, 5 RTK float, 6 INS
  uint8_t satellites = 0;
  float hdop = 0;
  float pdop = -1;             // GSA; -1 until a sentence arrives
  double latitude = 0;         // degrees, north positive
  double longitude = 0;        // degrees, east positive
  double altitude = 0;         // metres above mean sea level
  double geoidSeparation = 0;  // ellipsoidal height = altitude + geoidSeparation
  float diffAge = -1;          // seconds; ComNav reports 99 while no corrections arrive
  uint16_t stationId = 0;
  char utcTime[11] = "";       // hhmmss.ss
  float sigmaLat = -1;         // metres, 1 sigma (GST)
  float sigmaLon = -1;
  float sigmaAlt = -1;

  bool hasPosition() const { return quality != 0; }
};

// One sample of the IMUDATA binary log. This firmware's message ID is not the 2264 in the
// public K8/K9 manual, so any CRC-valid frame whose body is 7 doubles is treated as IMU.
struct K902Imu {
  uint16_t messageId = 0;
  double gpsSeconds = 0;
  double gyro[3] = {0, 0, 0};   // rad/s if the manual layout matches this firmware
  double accel[3] = {0, 0, 0};  // m/s^2, or g on some firmware builds
};

const char *k902QualityName(uint8_t quality);

// SinoGNSS (ComNav) K902 on one UART: sends ComNav ASCII commands, decodes NMEA and the IMUDATA
// binary log, and passes RTCM3 corrections through. Once a ComNav port is set to "interfacemode auto auto on" it accepts only
// RTCM, so every command batch is wrapped in "interfacemode compass compass on" ... "auto auto on".
class K902 {
 public:
  using SentenceHandler = void (*)(const char *sentence, size_t len);  // checksum-valid NMEA incl. CR LF
  using FixHandler = void (*)(const K902Fix &fix);                     // after every GGA
  using TextHandler = void (*)(const char *line);                      // command replies, ComNav # logs
  using ImuHandler = void (*)(const K902Imu &imu);                     // every CRC-valid IMUDATA frame
  using RtcmOutHandler = void (*)(const uint8_t *data, size_t len);    // complete RTCM3 frames from the board

  void begin(HardwareSerial &uart, int8_t rxPin, int8_t txPin, uint32_t baud,
             const char *const *setupCommands, size_t setupCount, bool rtcmInput);
  void poll();

  void configure();
  bool sendCommand(const char *command);
  bool acceptsCorrections() const;
  const char *portState() const;
  uint8_t pendingCommands() const { return queueCount_; }
  size_t freeCommandSlots() const { return kQueueLen - queueCount_; }
  size_t writeCorrections(const uint8_t *data, size_t len);

  void onSentence(SentenceHandler handler) { sentenceHandler_ = handler; }
  void onFix(FixHandler handler) { fixHandler_ = handler; }
  void onText(TextHandler handler) { textHandler_ = handler; }
  void onImu(ImuHandler handler) { imuHandler_ = handler; }
  void onRtcmOut(RtcmOutHandler handler) { rtcmOutHandler_ = handler; }

  const K902Fix &fix() const { return fix_; }
  bool dataFlowing() const;
  bool nmeaFlowing() const;
  uint32_t lastNmeaMs() const { return lastSentenceMs_; }
  uint32_t baud() const { return bauds_[baudIndex_]; }
  uint32_t correctionBytes() const { return rtcmBytes_; }
  uint32_t correctionFrames() const { return rtcm_.frames; }
  uint32_t correctionBasePos() const { return rtcm_.basePos; }
  uint32_t correctionMsm() const { return rtcm_.msm; }
  uint32_t correctionGps() const { return rtcm_.gpsMsm; }
  uint32_t correctionGlo() const { return rtcm_.gloMsm; }
  uint32_t correctionGal() const { return rtcm_.galMsm; }
  uint32_t correctionQzs() const { return rtcm_.qzsMsm; }
  uint32_t correctionBds() const { return rtcm_.bdsMsm; }
  uint16_t lastCorrectionType() const { return rtcm_.lastType; }
  uint32_t imuFrames() const { return imuFrames_; }
  uint16_t imuMessageId() const { return imuMessageId_; }
  uint16_t otherBinaryId() const { return otherBinaryId_; }
  uint16_t otherBinaryLength() const { return otherBinaryLen_; }

 private:
  static const size_t kLineMax = 512;
  static const size_t kBinaryMax = 512;
  static const size_t kMaxFields = 24;
  static const size_t kQueueLen = 24;
  static const size_t kCommandMax = 128;
  static const size_t kMaxBauds = 9;

  enum class PortMode : uint8_t { Unknown, Command, Differential };

  struct RtcmFramer {
    uint32_t frames = 0;
    uint16_t remaining = 0;  // payload + CRC bytes still expected
    uint16_t length = 0;
    uint8_t headerBytes = 0;
    uint8_t typeBytes = 0;
    uint16_t typeAcc = 0;
    uint32_t basePos = 0;   // 1005/1006
    uint32_t msm = 0;       // 1071-1230 constellation observations
    uint32_t gpsMsm = 0;    // 1071-1077
    uint32_t gloMsm = 0;    // 1081-1087
    uint32_t galMsm = 0;    // 1091-1097
    uint32_t qzsMsm = 0;    // 1111-1117
    uint32_t bdsMsm = 0;    // 1121-1127
    uint16_t lastType = 0;

    void push(uint8_t b);
    size_t bytesToFinish() const;
    void reset() { remaining = 0; length = 0; headerBytes = 0; typeBytes = 0; typeAcc = 0; }
  };

  void readUart(uint32_t now);
  bool feedBinary(uint8_t b);
  bool feedRtcmOut(uint8_t b);
  bool decodeShortImu(const uint8_t *msg, double *accel);
  void handleBinary(size_t total);
  void handleLine(uint32_t now);
  bool isSentence(const char *type) const;
  void parseGga(char **f, size_t n);
  void parseGst(char **f, size_t n);
  void parseGsa(char **f, size_t n);
  void checkWatchdog(uint32_t now);
  void serviceCommands(uint32_t now);
  void enterDifferential(uint32_t now);
  void leaveDifferentialMode();
  void transmit(const char *command);

  HardwareSerial *uart_ = nullptr;
  const char *const *setup_ = nullptr;
  size_t setupCount_ = 0;
  bool rtcmInput_ = false;

  uint32_t bauds_[kMaxBauds] = {};
  uint8_t baudCount_ = 0;
  uint8_t baudIndex_ = 0;
  uint8_t nextFallback_ = 1;
  uint8_t failedAttempts_ = 0;

  char line_[kLineMax];
  size_t lineLen_ = 0;
  bool lineOverflow_ = false;

  uint8_t bin_[kBinaryMax];
  size_t binLen_ = 0;
  uint8_t rtcmOut_[1029];
  size_t rtcmOutLen_ = 0;
  uint16_t rtcmOutLeft_ = 0;
  uint8_t rtcmOutHeader_ = 0;
  uint16_t rtcmOutLength_ = 0;
  uint32_t imuFrames_ = 0;
  uint16_t imuMessageId_ = 0;
  uint16_t otherBinaryId_ = 0;
  uint16_t otherBinaryLen_ = 0;
  int8_t imuAccelKind_ = -1;  // 0 = 3 floats, 1 = 3 doubles, inside a 48-byte IMUDATA body
  uint8_t imuAccelOff_ = 0;

  char queue_[kQueueLen][kCommandMax];
  uint8_t queueHead_ = 0;
  uint8_t queueCount_ = 0;
  PortMode portMode_ = PortMode::Unknown;
  uint32_t nextTxMs_ = 0;
  uint32_t lastTxMs_ = 0;
  uint32_t commandSinceMs_ = 0;

  uint32_t startedMs_ = 0;
  bool configured_ = false;
  uint32_t configuredMs_ = 0;
  bool dataSinceConfigure_ = false;
  uint32_t lastSentenceMs_ = 0;
  uint32_t lastBinaryMs_ = 0;

  RtcmFramer rtcm_;
  uint32_t rtcmBytes_ = 0;

  K902Fix fix_;
  SentenceHandler sentenceHandler_ = nullptr;
  FixHandler fixHandler_ = nullptr;
  TextHandler textHandler_ = nullptr;
  ImuHandler imuHandler_ = nullptr;
  RtcmOutHandler rtcmOutHandler_ = nullptr;
};
