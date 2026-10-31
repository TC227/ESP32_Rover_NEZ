#include "K902.h"

#include <cmath>

namespace {

const uint32_t kBootWaitMs = 2000;
const uint32_t kCommandGapMs = 150;
const uint32_t kDifferentialHoldMs = 400;  // let the last command land, then accept RTCM again
const uint32_t kCommandModeCapMs = 3000;   // never stay in command mode long enough for the RTK age to run away
const uint32_t kNoDataTimeoutMs = 4000;
const uint32_t kDataFlowingMs = 4000;
const uint8_t kAttemptsPerBaud = 2;
const uint32_t kFallbackBauds[] = {115200, 460800, 230400, 921600, 9600, 57600, 38400, 19200};
const uint16_t kImuBodyBytes = 56;   // manual layout: time + 3 gyro + 3 accel, all doubles
const uint16_t kImuShortBytes = 48;  // this K902 firmware: message 2264 is 48 bytes, not 56

// RTCM3 frame check (Qualcomm CRC-24Q over preamble, length and payload).
uint32_t crc24q(const uint8_t *data, size_t len) {
  uint32_t crc = 0;
  while (len--) {
    crc ^= (uint32_t)(*data++) << 16;
    for (int k = 0; k < 8; k++) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

// NovAtel-style CRC32 (no pre/post inversion), as used by ComNav logs.
uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0;
  while (len--) {
    uint32_t t = (crc ^ *data++) & 0xFF;
    for (int k = 0; k < 8; k++) t = (t & 1) ? (t >> 1) ^ 0xEDB88320u : t >> 1;
    crc = (crc >> 8) ^ t;
  }
  return crc;
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool checksumValid(const char *sentence) {
  uint8_t sum = 0;
  const char *p = sentence + 1;
  while (*p && *p != '*') sum ^= (uint8_t)*p++;
  if (*p != '*') return false;
  int hi = hexValue(p[1]);
  int lo = hexValue(p[2]);
  return hi >= 0 && lo >= 0 && sum == ((hi << 4) | lo);
}

bool printable(const char *s, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 0x20 || c > 0x7E) return false;
  }
  return true;
}

size_t splitFields(char *s, char **fields, size_t maxFields) {
  size_t n = 0;
  fields[n++] = s;
  for (char *p = s; *p && n < maxFields; ++p) {
    if (*p == ',') {
      *p = '\0';
      fields[n++] = p + 1;
    }
  }
  return n;
}

// NMEA ddmm.mmmm / dddmm.mmmm to signed degrees.
bool parseCoordinate(const char *value, const char *hemisphere, double *out) {
  if (!value[0] || !hemisphere[0]) return false;
  double raw = strtod(value, nullptr);
  double degrees = floor(raw / 100.0);
  double result = degrees + (raw - degrees * 100.0) / 60.0;
  *out = (hemisphere[0] == 'S' || hemisphere[0] == 'W') ? -result : result;
  return true;
}

float optionalFloat(const char *value) {
  return value[0] ? (float)atof(value) : -1.0f;
}

// Lower is better. A vector near 9.81 m/s^2 ranks ahead of one near 1 g.
bool gravityScore(double x, double y, double z, float *score) {
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return false;
  double n = sqrt(x * x + y * y + z * z);
  if (n > 7.0 && n < 13.0) {
    *score = (float)fabs(n - 9.80665);
    return true;
  }
  if (n > 0.7 && n < 1.4) {
    *score = 10.0f + (float)fabs(n - 1.0);
    return true;
  }
  return false;
}

void readAccel(const uint8_t *msg, int kind, uint8_t off, double *accel) {
  if (kind == 1) {
    memcpy(accel, msg + off, 24);
    return;
  }
  float f[3];
  memcpy(f, msg + off, 12);
  accel[0] = f[0];
  accel[1] = f[1];
  accel[2] = f[2];
}

}  // namespace

const char *k902QualityName(uint8_t quality) {
  switch (quality) {
    case 0: return "NOFIX";
    case 1: return "SINGLE";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "FIXED";
    case 5: return "FLOAT";
    case 6: return "INS";
    default: return "OTHER";
  }
}

void K902::RtcmFramer::push(uint8_t b) {
  if (remaining) {
    if (typeBytes < 2) {
      typeAcc = (uint16_t)((typeAcc << 8) | b);
      typeBytes++;
      if (typeBytes == 2) {
        lastType = typeAcc >> 4;
        if (lastType == 1005 || lastType == 1006) basePos++;
        else if (lastType >= 1071 && lastType <= 1077) { msm++; gpsMsm++; }
        else if (lastType >= 1081 && lastType <= 1087) { msm++; gloMsm++; }
        else if (lastType >= 1091 && lastType <= 1097) { msm++; galMsm++; }
        else if (lastType >= 1111 && lastType <= 1117) { msm++; qzsMsm++; }
        else if (lastType >= 1121 && lastType <= 1127) { msm++; bdsMsm++; }
        else if (lastType >= 1071 && lastType <= 1230) msm++;
      }
    }
    if (--remaining == 0) {
      frames++;
      typeBytes = 0;
      typeAcc = 0;
    }
    return;
  }
  if (headerBytes == 0) {
    if (b == 0xD3) headerBytes = 1;
  } else if (headerBytes == 1) {
    if (b & 0xFC) {
      headerBytes = 0;
    } else {
      length = (uint16_t)b << 8;
      headerBytes = 2;
    }
  } else {
    length |= b;
    remaining = length + 3;
    headerBytes = 0;
    typeBytes = 0;
    typeAcc = 0;
  }
}

size_t K902::RtcmFramer::bytesToFinish() const {
  if (remaining) return remaining;
  if (headerBytes == 1) return 2 + 3;
  if (headerBytes == 2) return 1 + length + 3;
  return 0;
}

void K902::begin(HardwareSerial &uart, int8_t rxPin, int8_t txPin, uint32_t baud,
                 const char *const *setupCommands, size_t setupCount, bool rtcmInput) {
  uart_ = &uart;
  setup_ = setupCommands;
  setupCount_ = setupCount;
  rtcmInput_ = rtcmInput;

  baudCount_ = 0;
  bauds_[baudCount_++] = baud;
  for (uint32_t b : kFallbackBauds) {
    if (b != baud && baudCount_ < kMaxBauds) bauds_[baudCount_++] = b;
  }
  baudIndex_ = 0;

  uart.setRxBufferSize(2048);
  uart.setTxBufferSize(2048);
  uart.begin(baud, SERIAL_8N1, rxPin, txPin);
  startedMs_ = millis();
}

void K902::poll() {
  uint32_t now = millis();
  readUart(now);
  if (!configured_) {
    if (now - startedMs_ >= kBootWaitMs) configure();
  } else {
    checkWatchdog(now);
  }
  serviceCommands(now);
}

void K902::configure() {
  uint32_t now = millis();
  // unlogall while GGA/IMU are already arriving is what knocks COM2 back to
  // command mode and chops the RTCM stream. One setup is enough until a real silence.
  // IMU binary alone does not count: if GGA stopped, the logs have to be re-sent.
  if (configured_ && (nmeaFlowing() || now - configuredMs_ < 20000)) return;
  bool first = !configured_;
  configured_ = true;
  configuredMs_ = millis();
  dataSinceConfigure_ = false;
  portMode_ = PortMode::Unknown;
  for (size_t i = 0; i < setupCount_; i++) {
    // Retries must not undo a base session ("fix none") or chop RTCM output ("unlogall").
    if (!first && (strncmp(setup_[i], "unlogall", 8) == 0 || strcmp(setup_[i], "fix none") == 0)) continue;
    if (!sendCommand(setup_[i])) {
      Serial.printf("K902 setup queue full at %s\n", setup_[i]);
      break;
    }
  }
  Serial.println(first ? "K902 setup" : "K902 setup retry");
}

bool K902::sendCommand(const char *command) {
  if (!command[0] || queueCount_ >= kQueueLen) return false;
  strlcpy(queue_[(queueHead_ + queueCount_) % kQueueLen], command, kCommandMax);
  queueCount_++;
  return true;
}

bool K902::acceptsCorrections() const {
  return rtcmInput_ && portMode_ == PortMode::Differential && queueCount_ == 0;
}

const char *K902::portState() const {
  switch (portMode_) {
    case PortMode::Command: return "CMD";
    case PortMode::Differential: return "DIFF";
    default: return "UNK";
  }
}

size_t K902::writeCorrections(const uint8_t *data, size_t len) {
  if (!acceptsCorrections()) return 0;
  uart_->write(data, len);
  for (size_t i = 0; i < len; i++) rtcm_.push(data[i]);
  rtcmBytes_ += len;
  return len;
}

bool K902::nmeaFlowing() const {
  return lastSentenceMs_ != 0 && millis() - lastSentenceMs_ < kDataFlowingMs;
}

bool K902::dataFlowing() const {
  // Prefer NMEA; IMU alone means the board is alive but the solution may be stale.
  if (nmeaFlowing()) return true;
  return lastBinaryMs_ != 0 && millis() - lastBinaryMs_ < kDataFlowingMs;
}

bool K902::feedRtcmOut(uint8_t b) {
  if (rtcmOutLen_ == 0 && b != 0xD3) return false;
  if (rtcmOutLen_ == 0) {
    rtcmOut_[0] = b;
    rtcmOutLen_ = 1;
    rtcmOutHeader_ = 1;
    rtcmOutLeft_ = 0;
    rtcmOutLength_ = 0;
    return true;
  }
  if (rtcmOutLen_ < sizeof(rtcmOut_)) rtcmOut_[rtcmOutLen_++] = b;
  if (rtcmOutLeft_) {
    rtcmOutLeft_--;
  } else if (rtcmOutHeader_ == 1) {
    if (b & 0xFC) {
      rtcmOutLen_ = 0;
      rtcmOutHeader_ = 0;
      return false;
    }
    rtcmOutLength_ = (uint16_t)b << 8;
    rtcmOutHeader_ = 2;
  } else if (rtcmOutHeader_ == 2) {
    rtcmOutLength_ |= b;
    rtcmOutLeft_ = rtcmOutLength_ + 3;
    rtcmOutHeader_ = 0;
    if (rtcmOutLength_ > 1023 || rtcmOutLen_ + rtcmOutLeft_ > sizeof(rtcmOut_)) {
      rtcmOutLen_ = 0;
      rtcmOutLeft_ = 0;
      return false;
    }
  }
  if (rtcmOutLeft_ == 0 && rtcmOutHeader_ == 0 && rtcmOutLen_ > 0) {
    size_t body = rtcmOutLen_ - 3;
    uint32_t want = ((uint32_t)rtcmOut_[body] << 16) | ((uint32_t)rtcmOut_[body + 1] << 8) | rtcmOut_[body + 2];
    if (rtcmOutHandler_ && crc24q(rtcmOut_, body) == want) rtcmOutHandler_(rtcmOut_, rtcmOutLen_);
    rtcmOutLen_ = 0;
  }
  return true;
}

void K902::readUart(uint32_t now) {
  while (uart_->available() > 0) {
    uint8_t b = (uint8_t)uart_->read();
    if (binLen_ > 0 && feedBinary(b)) continue;
    if (rtcmOutLen_ > 0 || (b == 0xD3 && lineLen_ == 0)) {
      if (feedRtcmOut(b)) {
        lineLen_ = 0;
        continue;
      }
    }
    if (b == 0xAA) {  // binary sync byte, never part of ASCII output
      bin_[0] = b;
      binLen_ = 1;
      lineLen_ = 0;
      continue;
    }
    char c = (char)b;
    if (c == '$') {
      lineLen_ = 0;
      lineOverflow_ = false;
    }
    if (c == '\n') {
      if (lineLen_ > 0 && !lineOverflow_) handleLine(now);
      lineLen_ = 0;
      lineOverflow_ = false;
    } else if (c != '\r') {
      if (lineLen_ < kLineMax - 3) {
        line_[lineLen_++] = c;
      } else {
        lineOverflow_ = true;
      }
    }
  }
}

// Binary frame: AA 44 12, header length, message ID (4-5), message length (8-9), header rest,
// message, CRC32. Returns false when b turns out not to belong to a frame.
bool K902::feedBinary(uint8_t b) {
  if ((binLen_ == 1 && b != 0x44) || (binLen_ == 2 && b != 0x12)) {
    binLen_ = 0;
    return false;
  }
  bin_[binLen_++] = b;
  if (binLen_ < 10) return true;
  size_t headerLen = bin_[3];
  size_t total = headerLen + (bin_[8] | (bin_[9] << 8)) + 4;
  if (headerLen < 10 || total > kBinaryMax) {
    binLen_ = 0;
  } else if (binLen_ == total) {
    handleBinary(total);
    binLen_ = 0;
  }
  return true;
}

bool K902::decodeShortImu(const uint8_t *msg, double *accel) {
  if (imuAccelKind_ >= 0) {
    readAccel(msg, imuAccelKind_, imuAccelOff_, accel);
    return true;
  }
  float best = 1.0e9f;
  int kind = -1;
  uint8_t off = 0;
  for (uint8_t i = 0; i + 24 <= kImuShortBytes; i += 8) {
    double a[3];
    readAccel(msg, 1, i, a);
    float score;
    if (gravityScore(a[0], a[1], a[2], &score) && score < best) {
      best = score;
      kind = 1;
      off = i;
    }
  }
  for (uint8_t i = 0; i + 12 <= kImuShortBytes; i += 4) {
    double a[3];
    readAccel(msg, 0, i, a);
    float score;
    if (gravityScore(a[0], a[1], a[2], &score) && score < best) {
      best = score;
      kind = 0;
      off = i;
    }
  }
  if (kind < 0) return false;
  imuAccelKind_ = (int8_t)kind;
  imuAccelOff_ = off;
  readAccel(msg, kind, off, accel);
  return true;
}

void K902::handleBinary(size_t total) {
  uint32_t crc;
  memcpy(&crc, bin_ + total - 4, 4);
  if (crc32(bin_, total - 4) != crc) return;
  uint16_t id = bin_[4] | (bin_[5] << 8);
  const uint8_t *msg = bin_ + bin_[3];
  size_t msgLen = total - 4 - bin_[3];

  K902Imu imu;
  imu.messageId = id;
  bool ok = false;
  if (msgLen >= kImuBodyBytes) {
    memcpy(&imu.gpsSeconds, msg, 8);
    memcpy(imu.gyro, msg + 8, 24);
    memcpy(imu.accel, msg + 32, 24);
    ok = true;
  } else if (msgLen == kImuShortBytes) {
    ok = decodeShortImu(msg, imu.accel);
  }
  if (!ok) {
    otherBinaryId_ = id;
    otherBinaryLen_ = (uint16_t)msgLen;
    return;
  }
  imuMessageId_ = id;
  imuFrames_++;
  lastBinaryMs_ = millis();
  if (imuHandler_) imuHandler_(imu);
}

bool K902::isSentence(const char *type) const {
  return lineLen_ > 7 && line_[6] == ',' && strncmp(line_ + 3, type, 3) == 0;
}

void K902::handleLine(uint32_t now) {
  line_[lineLen_] = '\0';
  char *star = line_[0] == '$' ? strchr(line_, '*') : nullptr;
  if (star == nullptr) {
    if (textHandler_ && printable(line_, lineLen_)) textHandler_(line_);
    return;
  }
  if (!checksumValid(line_)) return;

  lastSentenceMs_ = now;
  dataSinceConfigure_ = true;
  failedAttempts_ = 0;

  if (sentenceHandler_) {
    memcpy(line_ + lineLen_, "\r\n", 3);
    sentenceHandler_(line_, lineLen_ + 2);
    line_[lineLen_] = '\0';
  }

  bool gga = isSentence("GGA");
  bool gst = isSentence("GST");
  bool gsa = isSentence("GSA");
  if (!gga && !gst && !gsa) {
    bool nmeaData = line_[1] == 'G' || (line_[1] == 'B' && line_[2] == 'D');
    if (textHandler_ && !nmeaData) textHandler_(line_);
    return;
  }

  *star = '\0';
  char *fields[kMaxFields];
  size_t n = splitFields(line_, fields, kMaxFields);
  if (gga) {
    parseGga(fields, n);
  } else if (gst) {
    parseGst(fields, n);
  } else {
    parseGsa(fields, n);
  }
}

void K902::parseGga(char **f, size_t n) {
  if (n < 14) return;
  strlcpy(fix_.utcTime, f[1], sizeof(fix_.utcTime));
  fix_.quality = (uint8_t)atoi(f[6]);
  fix_.satellites = (uint8_t)atoi(f[7]);
  fix_.hdop = (float)atof(f[8]);
  if (fix_.quality != 0 && parseCoordinate(f[2], f[3], &fix_.latitude) &&
      parseCoordinate(f[4], f[5], &fix_.longitude)) {
    fix_.altitude = atof(f[9]);
    fix_.geoidSeparation = atof(f[11]);
  } else {
    fix_.quality = 0;
  }
  fix_.diffAge = optionalFloat(f[13]);
  fix_.stationId = n > 14 ? (uint16_t)atoi(f[14]) : 0;
  if (fixHandler_) fixHandler_(fix_);
}

void K902::parseGst(char **f, size_t n) {
  if (n < 9) return;
  fix_.sigmaLat = optionalFloat(f[6]);
  fix_.sigmaLon = optionalFloat(f[7]);
  fix_.sigmaAlt = optionalFloat(f[8]);
}

void K902::parseGsa(char **f, size_t n) {
  if (n < 6) return;
  size_t i = n >= 18 ? 15 : n - 3;
  float pdop = optionalFloat(f[i]);
  float hdop = optionalFloat(f[i + 1]);
  if (pdop > 0) fix_.pdop = pdop;
  if (hdop > 0) fix_.hdop = hdop;
}

// Re-sends the setup when the K902 goes quiet (it booted after us or was power-cycled), and moves
// on to the next baud rate after kAttemptsPerBaud silent attempts.
void K902::checkWatchdog(uint32_t now) {
  // NMEA silence triggers recovery. IMU-only traffic must not block a re-setup when GGA dies.
  if (nmeaFlowing()) return;
  if (queueCount_ > 0 || portMode_ != PortMode::Differential) return;
  if (now - configuredMs_ < 20000) return;
  uint32_t since = lastSentenceMs_ != 0 ? lastSentenceMs_ : configuredMs_;
  if (now - since < kNoDataTimeoutMs) return;
  // A CRC-valid IMU frame proves the baud is right even when NMEA is missing.
  if (!dataSinceConfigure_ && !dataFlowing() && ++failedAttempts_ >= kAttemptsPerBaud) {
    failedAttempts_ = 0;
    // Schedule: preferred(0) → fallback1 → preferred → fallback2 → … so a late-powered
    // K902 on the wired baud recovers without waiting through every exotic rate first.
    if (baudIndex_ == 0 && baudCount_ > 1) {
      if (nextFallback_ < 1 || nextFallback_ >= baudCount_) nextFallback_ = 1;
      baudIndex_ = nextFallback_++;
    } else {
      baudIndex_ = 0;
    }
    uart_->updateBaudRate(bauds_[baudIndex_]);
    lineLen_ = 0;
    Serial.printf("K902 baud try %lu\n", (unsigned long)bauds_[baudIndex_]);
  }
  configure();
}

void K902::enterDifferential(uint32_t now) {
  queueCount_ = 0;
  transmit("interfacemode auto auto on");
  portMode_ = PortMode::Differential;
  configuredMs_ = now;
  Serial.println("K902 port DIFF, accepting RTCM");
}

void K902::serviceCommands(uint32_t now) {
  if (!configured_ || (int32_t)(now - nextTxMs_) < 0) return;
  if (rtcmInput_ && portMode_ == PortMode::Command && queueCount_ == 0 && commandSinceMs_ != 0 &&
      now - commandSinceMs_ >= kCommandModeCapMs) {
    enterDifferential(now);
  } else if (portMode_ != PortMode::Command && (queueCount_ > 0 || portMode_ == PortMode::Unknown)) {
    leaveDifferentialMode();
    commandSinceMs_ = now;
  } else if (queueCount_ > 0) {
    transmit(queue_[queueHead_]);
    queueHead_ = (queueHead_ + 1) % kQueueLen;
    queueCount_--;
  } else if (rtcmInput_ && portMode_ == PortMode::Command && now - lastTxMs_ >= kDifferentialHoldMs) {
    enterDifferential(now);
  } else {
    return;
  }
  lastTxMs_ = now;
  nextTxMs_ = now + kCommandGapMs;
}

void K902::leaveDifferentialMode() {
  // The K902 would swallow the escape command as payload of a half-sent RTCM frame, so pad it out.
  static const uint8_t kZeros[64] = {};
  for (size_t left = rtcm_.bytesToFinish(); left > 0;) {
    size_t n = left < sizeof(kZeros) ? left : sizeof(kZeros);
    uart_->write(kZeros, n);
    left -= n;
  }
  rtcm_.reset();
  transmit("interfacemode compass compass on");
  portMode_ = PortMode::Command;
}

void K902::transmit(const char *command) {
  uart_->write((const uint8_t *)command, strlen(command));
  uart_->write((const uint8_t *)"\r\n", 2);
}
