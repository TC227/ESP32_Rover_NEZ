#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <string>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/stream_buffer.h>
#include <esp_system.h>
#include <Preferences.h>

#include "K902.h"

// ESP32 UART2 <-> K902 UART2 (COM2), 3.3 V TTL (never wire its RS-232 DB9 directly):
// K902 TXD -> GPIO16 (RX2), K902 RXD <- GPIO17 (TX2), GND <-> GND.
#define K902_RX_PIN 16
#define K902_TX_PIN 17
#define K902_BAUD 115200
#define K902_PORT "COM2"   // K902 port those wires go to
#define K902_RTCM_INPUT 1  // RTCM3 written by the phone to the RX characteristic is fed to the K902

#define BLE_DEVICE_NAME "GNSS"  // BLE name for this app, and the Classic name Survey Master pairs with
#define BLE_SEND_NMEA 1    // every NMEA sentence on BLE; SW Maps needs GGA and RMC for a position
#define BLE_SEND_PRTK_S 1  // "$PRTK,<fix>,<sats>" 1 Hz + "$S,<seq>,<dN cm>,<dE cm>" movement per fix
#define TILT_ENABLED 1     // pole bubble from the K902 IMU: status line + "$TILT,<pitch>,<roll>"
// Bubble snappiness: higher blend / lower thresholds = follows the pole faster (more IMU noise).
static const float kTiltAccelBlend = 0.88f;   // fraction of each new accel sample kept (was 0.70)
static const float kTiltShowEpsDeg = 0.04f;   // update shown angles above this delta (was 0.15°)
// Keep tilt sparse: phone→ESP RTCM writes share the same radio. Flooding notifies
// starves WRITE_NR and the K902 RTK LED stays dark (phone writes stay at 0).
static const unsigned kTiltBleGapMs = 100;    // ~10 Hz max
static const unsigned kTiltBleRetryMs = 40;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// Dedicated notify lane for the pole bubble — not shared with NMEA / RTCM on TX.
#define CHARACTERISTIC_UUID_TILT "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
// Binary tilt packet: [0x1F][pitch_cdeg int16 LE][roll_cdeg int16 LE]
static const uint8_t kTiltPacketTag = 0x1F;

// Sent to the K902 after boot and again whenever its output stops.
const char *const kK902Setup[] = {
  "unlogall " K902_PORT,
  "fix none",  // a base session left by a previous ESP32 run must not freeze the rover position
  "unlockoutall",
  "rtkobsmode 0",
  "set appscene survey",
  "log gpgga ontime 0.2",
  "log gprmc ontime 1",
  "log gpgst ontime 1",
  "log gpgsa ontime 1",
  "log glgsa ontime 1",
  "log gagsa ontime 1",
  "log gbgsa ontime 1",
  "log gpgsv ontime 5",
  "log glgsv ontime 5",
  "log gagsv ontime 5",
  "log gbgsv ontime 5",
#if TILT_ENABLED
  "log " K902_PORT " imudatab ontime 0.1",
#endif
};

K902 gnss;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
BLECharacteristic *pTiltCharacteristic = NULL;
volatile bool deviceConnected = false;
volatile unsigned long bleConnectedAtMs = 0;
volatile bool phoneWriteAnnounced = false;
volatile bool phoneWriteWarned = false;
volatile uint16_t peerMtu = 23;
volatile bool connectEvent = false;
volatile bool disconnectEvent = false;
volatile bool mtuEvent = false;
volatile uint16_t disconnectReason = 0;
volatile bool paramsEvent = false;
volatile bool paramsRejected = false;
volatile bool advFailedEvent = false;
volatile bool authEvent = false;
volatile bool authOk = false;
volatile uint8_t authReason = 0;
volatile uint16_t connInterval = 0;  // 1.25 ms units
volatile uint16_t connLatency = 0;
volatile uint16_t connTimeout = 0;   // 10 ms units
esp_bd_addr_t peerAddr = {0};
bool oldDeviceConnected = false;
bool advertisePending = false;
bool paramsRequested = false;
unsigned long disconnectedAt = 0;
unsigned long phoneLostAt = 0;
unsigned long connectedAt = 0;
StreamBufferHandle_t rtcmBuffer = NULL;
SemaphoreHandle_t rtcmMux = NULL;
const size_t kPhoneCmdMax = 160;
QueueHandle_t phoneCmdQueue = NULL;  // BLE task -> loop, whole "$ICMD/$BCMD" lines
char phoneCmdAcc[kPhoneCmdMax];
size_t phoneCmdAccLen = 0;
volatile uint32_t bleWriteCount = 0;
volatile uint32_t bleWriteBytes = 0;
volatile uint32_t sppBytes = 0;
volatile bool sppUpEvent = false;
volatile bool sppDownEvent = false;
volatile bool sppCongested = false;
volatile bool baseActive = false;
unsigned long lastBstatMs = 0;
volatile bool rawLogActive = false;  // phone is recording the K902's own RTCM observations
unsigned long lastRtcmOutMs = 0;
unsigned long lastRawArmMs = 0;
unsigned long lastLstatMs = 0;

void handlePhoneCommand(char *cmd);

BluetoothSerial SerialBT;

bool rawEcho = false;
bool quietStatus = false;
size_t consoleLen = 0;
uint8_t stepSeq = 0;
bool originSet = false;
double originLat = 0;
double originLon = 0;
double cmPerDegLat = 0;
double cmPerDegLon = 0;
bool stepBaseSet = false;
long stepN = 0;
long stepE = 0;
unsigned long lastStatus = 0;

Preferences prefs;
float levelRef[3] = {0, 0, 1};  // unit gravity vector with the pole held vertical ("!level")
float gravity[3] = {0, 0, 0};   // low-passed accelerometer, m/s^2
bool gravitySet = false;
float tiltDeg = 0;
float shownPitch = 0;
float shownRoll = 0;
bool tiltFresh = false;
unsigned long lastImuMs = 0;
unsigned long lastTiltBleMs = 0;
unsigned long lastTiltAttemptMs = 0;
unsigned long tiltTx = 0;
unsigned long tiltDrop = 0;
unsigned long lastGgaBleMs = 0;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server, esp_ble_gatts_cb_param_t *param) {
    Serial.println("BLE Server: onConnect called");
    deviceConnected = true;
    bleConnectedAtMs = millis();
    phoneWriteAnnounced = false;
    phoneWriteWarned = false;
    memcpy(peerAddr, param->connect.remote_bda, sizeof(peerAddr));
    connInterval = param->connect.conn_params.interval;
    connLatency = param->connect.conn_params.latency;
    connTimeout = param->connect.conn_params.timeout;
    connectEvent = true;
    BLEDevice::getAdvertising()->stop();
    BLE2902 *ccc = (BLE2902 *)pTxCharacteristic->getDescriptorByUUID((uint16_t)0x2902);
    if (ccc != NULL) ccc->setNotifications(true);
    if (pTiltCharacteristic != NULL) {
      BLE2902 *tiltCcc = (BLE2902 *)pTiltCharacteristic->getDescriptorByUUID((uint16_t)0x2902);
      if (tiltCcc != NULL) tiltCcc->setNotifications(true);
    }
  }
  void onDisconnect(BLEServer *server, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = false;
    peerMtu = 23;
    disconnectReason = param->disconnect.reason;
    disconnectEvent = true;
  }
  void onMtuChanged(BLEServer *server, esp_ble_gatts_cb_param_t *param) {
    peerMtu = param->mtu.mtu;
    mtuEvent = true;
  }
};

bool allCommandAscii(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t c = data[i];
    if (c != '\r' && c != '\n' && (c < 0x20 || c > 0x7E)) return false;
  }
  return true;
}

// Only these reach the command path; anything else on the RX characteristic is RTCM.
bool commandPrefixOk(const char *s, size_t len) {
  static const char *const kPrefixes[] = {"$ICMD,", "$BCMD,", "$LCMD,"};
  for (const char *p : kPrefixes) {
    size_t n = strlen(p);
    if (strncmp(s, p, len < n ? len : n) == 0) return true;
  }
  return false;
}

// Returns false when the bytes are not a phone command, so the caller forwards them as RTCM.
bool appendPhoneCommandBytes(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t c = data[i];
    if (c == '\r' || c == '\n') {
      if (phoneCmdAccLen > 0) {
        phoneCmdAcc[phoneCmdAccLen] = '\0';
        if (phoneCmdQueue && xQueueSend(phoneCmdQueue, phoneCmdAcc, 0) != pdTRUE) {
          Serial.println("Phone command queue full");
        }
        phoneCmdAccLen = 0;
      }
      continue;
    }
    if (phoneCmdAccLen >= kPhoneCmdMax - 1) {
      phoneCmdAccLen = 0;
      return false;
    }
    phoneCmdAcc[phoneCmdAccLen++] = (char)c;
    if (!commandPrefixOk(phoneCmdAcc, phoneCmdAccLen)) {
      phoneCmdAccLen = 0;
      return false;
    }
  }
  return true;
}

// All-or-nothing so a full buffer never splices half a frame into the next one.
void rtcmPush(const void *data, size_t len) {
  if (rtcmBuffer == NULL || rtcmMux == NULL || len == 0 || baseActive) return;
  if (xSemaphoreTake(rtcmMux, pdMS_TO_TICKS(5)) != pdTRUE) return;
  if (xStreamBufferSpacesAvailable(rtcmBuffer) >= len) xStreamBufferSend(rtcmBuffer, data, len, 0);
  xSemaphoreGive(rtcmMux);
}

// Writes are ingested in onGatts (ESP_GATTS_WRITE_EVT). Keeping an empty callback
// avoids double-pushing the same RTCM chunk into the K902.
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { (void)c; }
};

void onGap(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) advFailedEvent = true;
      break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      if (param->update_conn_params.status == ESP_BT_STATUS_SUCCESS) {
        connInterval = param->update_conn_params.conn_int;
        connLatency = param->update_conn_params.latency;
        connTimeout = param->update_conn_params.timeout;
      } else {
        paramsRejected = true;
      }
      paramsEvent = true;
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      authOk = param->ble_security.auth_cmpl.success;
      authReason = param->ble_security.auth_cmpl.fail_reason;
      authEvent = true;
      break;
    default:
      break;
  }
}

volatile bool lastNotifyOk = false;
volatile bool bleCongested = false;
volatile uint32_t bleNotifyFails = 0;
unsigned long bleCongestedSinceMs = 0;
float lastTiltSentPitch = 1e9f;
float lastTiltSentRoll = 1e9f;
unsigned long lastTiltAsciiMs = 0;

class TxCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic *characteristic, Status status, uint32_t code) override {
    (void)characteristic;
    (void)code;
    lastNotifyOk = status == SUCCESS_NOTIFY;
  }
};

void serviceBleCongestion(unsigned long now);

bool bleLinkReady() {
  if (!deviceConnected || pTxCharacteristic == NULL) return false;
  if (bleCongested) serviceBleCongestion(millis());
  return !bleCongested;
}

bool bleNotifyRaw(const uint8_t *data, size_t len) {
  if (!bleLinkReady() || len == 0) return false;
  size_t chunk = peerMtu > 3 ? (size_t)(peerMtu - 3) : 20;
  size_t sent = 0;
  while (len > 0) {
    if (bleCongested) return false;
    size_t n = len < chunk ? len : chunk;
    pTxCharacteristic->setValue(const_cast<uint8_t *>(data), n);
    lastNotifyOk = false;
    pTxCharacteristic->notify();
    // Short lines ($TILT, $S): one retry without delay. Long NMEA may wait 2 ms once.
    if (!lastNotifyOk) {
      if (n > 32) delay(2);
      if (bleCongested) return false;
      lastNotifyOk = false;
      pTxCharacteristic->notify();
    }
    if (!lastNotifyOk) {
      if (sent > 0) {
        uint8_t nl = '\n';
        pTxCharacteristic->setValue(&nl, 1);
        pTxCharacteristic->notify();
      }
      return false;
    }
    data += n;
    len -= n;
    sent += n;
  }
  return true;
}

bool bleSend(const char *data, size_t len) {
  return bleNotifyRaw(reinterpret_cast<const uint8_t *>(data), len);
}

// Compact tilt on its own characteristic — avoids waiting behind NMEA/RTCM on TX.
bool bleSendTiltBinary(float pitchDeg, float rollDeg) {
  if (!deviceConnected || pTiltCharacteristic == NULL || bleCongested) return false;
  auto toCdeg = [](float deg) -> int16_t {
    long v = lroundf(deg * 100.0f);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
  };
  uint8_t pkt[5];
  pkt[0] = kTiltPacketTag;
  int16_t p = toCdeg(pitchDeg);
  int16_t r = toCdeg(rollDeg);
  pkt[1] = (uint8_t)(p & 0xFF);
  pkt[2] = (uint8_t)((p >> 8) & 0xFF);
  pkt[3] = (uint8_t)(r & 0xFF);
  pkt[4] = (uint8_t)((r >> 8) & 0xFF);
  pTiltCharacteristic->setValue(pkt, sizeof(pkt));
  lastNotifyOk = false;
  pTiltCharacteristic->notify();
  return lastNotifyOk;
}

// RTCM from the K902 (base mode, raw log) goes up as binary packets, one notification each:
//   [0x1E][frame seq][part index, bit 7 = last part][payload]
// The phone joins parts by seq and checks CRC-24Q, so a lost packet costs one frame and never
// corrupts the NMEA text stream. Packets wait in a queue and leave one at a time with a gap,
// because a burst of notifications overflows the BLE controller and gets dropped silently.
const uint8_t kRtcmPacketTag = 0x1E;
const size_t kRawSlots = 32;
const size_t kRawSlotMax = 244;
const unsigned long kRawTxGapMs = 8;
const unsigned long kRawTxRetryMs = 40;
struct RawPacket {
  uint8_t len;
  uint8_t data[kRawSlotMax];
};
RawPacket rawQueue[kRawSlots];
uint8_t rawHead = 0;
uint8_t rawCount = 0;
uint8_t rawFrameSeq = 0;
unsigned long rawNextTxMs = 0;
uint32_t rawFramesQueued = 0;
uint32_t rawFramesDropped = 0;

volatile uint16_t rxCharHandle = 0;
volatile uint16_t lastGattsWriteHandle = 0;
volatile uint16_t lastGattsWriteLen = 0;
volatile uint8_t lastGattsWriteByte = 0;
volatile uint32_t gattsWriteSeen = 0;
volatile bool phoneFirstWritePending = false;

void ingestPhoneWrite(const uint8_t *data, size_t len) {
  if (len == 0 || data == NULL) return;
  bleWriteCount++;
  bleWriteBytes += len;
  if (!phoneWriteAnnounced) {
    phoneWriteAnnounced = true;
    phoneFirstWritePending = true;
    lastGattsWriteByte = data[0];
    lastGattsWriteLen = (uint16_t)len;
  }
  if ((phoneCmdAccLen > 0 || data[0] == '$') && allCommandAscii(data, len)) {
    if (appendPhoneCommandBytes(data, len)) return;
  } else {
    phoneCmdAccLen = 0;
  }
  rtcmPush(data, len);
}

void onGatts(esp_gatts_cb_event_t event, esp_gatt_if_t gattsIf, esp_ble_gatts_cb_param_t *param) {
  (void)gattsIf;
  if (event == ESP_GATTS_CONGEST_EVT) {
    const bool next = param->congest.congested;
    if (next && !bleCongested) bleCongestedSinceMs = millis();
    if (!next) bleCongestedSinceMs = 0;
    bleCongested = next;
  } else if (event == ESP_GATTS_CONF_EVT && param->conf.status != ESP_GATT_OK) {
    bleNotifyFails++;
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    bleCongested = false;
    bleCongestedSinceMs = 0;
  } else if (event == ESP_GATTS_WRITE_EVT) {
    // Ingest on the GATTS path — never Serial-print here (BLE task; printing reboots).
    lastGattsWriteHandle = param->write.handle;
    lastGattsWriteLen = param->write.len;
    gattsWriteSeen++;
    if (!param->write.is_prep && param->write.len > 0 && param->write.value != NULL &&
        rxCharHandle != 0 && param->write.handle == rxCharHandle) {
      ingestPhoneWrite(param->write.value, param->write.len);
    }
  }
}

// If we stop sending while congested, the stack may never deliver a clear event.
void serviceBleCongestion(unsigned long now) {
  if (!bleCongested || bleCongestedSinceMs == 0) return;
  if (now - bleCongestedSinceMs < 300) return;
  bleCongested = false;
  bleCongestedSinceMs = 0;
}

void clearRawQueue() {
  rawHead = 0;
  rawCount = 0;
}

// Whole frame or nothing: half a frame in the queue would only fail CRC on the phone.
bool queueRtcmToPhone(const uint8_t *data, size_t len) {
  size_t payload = peerMtu > 3 ? (size_t)(peerMtu - 3) : 20;
  if (payload < 4) payload = 20;
  if (payload > kRawSlotMax) payload = kRawSlotMax;
  const size_t room = payload - 3;
  const size_t parts = (len + room - 1) / room;
  if (len == 0 || parts > 127 || kRawSlots - rawCount < parts) {
    rawFramesDropped++;
    return false;
  }
  const uint8_t seq = rawFrameSeq++;
  for (size_t p = 0; p < parts; p++) {
    const size_t off = p * room;
    const size_t n = len - off < room ? len - off : room;
    RawPacket &pkt = rawQueue[(rawHead + rawCount) % kRawSlots];
    pkt.data[0] = kRtcmPacketTag;
    pkt.data[1] = seq;
    pkt.data[2] = (uint8_t)p | (p + 1 == parts ? 0x80 : 0);
    memcpy(pkt.data + 3, data + off, n);
    pkt.len = (uint8_t)(n + 3);
    rawCount++;
  }
  rawFramesQueued++;
  return true;
}

void serviceRawTx(unsigned long now) {
  if (rawCount == 0) return;
  if (!deviceConnected || pTxCharacteristic == NULL) {
    clearRawQueue();
    return;
  }
  if (bleCongested || (long)(now - rawNextTxMs) < 0) return;
  RawPacket &pkt = rawQueue[rawHead];
  pTxCharacteristic->setValue(pkt.data, pkt.len);
  lastNotifyOk = false;
  pTxCharacteristic->notify();
  if (!lastNotifyOk) {
    rawNextTxMs = now + kRawTxRetryMs;
    return;
  }
  rawHead = (rawHead + 1) % kRawSlots;
  rawCount--;
  rawNextTxMs = now + kRawTxGapMs;
}

void setOrigin(const K902Fix &fix) {
  const double a = 6378137.0;
  const double e2 = 6.69437999014e-3;
  double phi = fix.latitude * DEG_TO_RAD;
  double w = sqrt(1.0 - e2 * sin(phi) * sin(phi));
  cmPerDegLat = a * (1.0 - e2) / (w * w * w) * DEG_TO_RAD * 100.0;
  cmPerDegLon = a / w * cos(phi) * DEG_TO_RAD * 100.0;
  originLat = fix.latitude;
  originLon = fix.longitude;
  originSet = true;
}

void onGnssFix(const K902Fix &fix) {
  if (!BLE_SEND_PRTK_S || !fix.hasPosition() || !oldDeviceConnected) return;
  if (!originSet) setOrigin(fix);
  long n = lround((fix.latitude - originLat) * cmPerDegLat);
  long e = lround((fix.longitude - originLon) * cmPerDegLon);
  if (!stepBaseSet) {
    stepN = n;
    stepE = e;
    stepBaseSet = true;
    return;
  }
  // Each message must fit one 20-byte notification; bigger jumps carry over to the next fixes.
  long dN = constrain(n - stepN, -9999L, 9999L);
  long dE = constrain(e - stepE, -9999L, 9999L);
  char line[20];
  snprintf(line, sizeof(line), "$S,%u,%ld,%ld\n", stepSeq, dN, dE);
  if (!bleSend(line, strlen(line))) return;
  stepN += dN;
  stepE += dE;
  stepSeq++;
}

char pendingGsa[256];
char pendingGst[160];
size_t pendingGsaLen = 0;
size_t pendingGstLen = 0;

bool sentenceType(const char *sentence, size_t len, const char *type) {
  return len >= 8 && sentence[0] == '$' && memcmp(sentence + 3, type, 4) == 0;
}

// `$GPGSV,3,1,11,...` — page 1 carries that constellation's in-view total.
bool gsvFirstPage(const char *sentence, size_t len) {
  int commas = 0;
  for (size_t i = 0; i < len; i++) {
    if (sentence[i] != ',') continue;
    if (++commas == 2) {
      return i + 1 < len && sentence[i + 1] == '1' && (i + 2 >= len || sentence[i + 2] == ',');
    }
  }
  return false;
}

void holdLine(char *dest, size_t cap, size_t *held, const char *sentence, size_t len) {
  if (len >= cap) len = cap - 1;
  memcpy(dest, sentence, len);
  dest[len] = '\0';
  *held = len;
}

void flushQualityLines() {
  if (!deviceConnected) return;
  if (pendingGsaLen) {
    bleSend(pendingGsa, pendingGsaLen);
    pendingGsaLen = 0;
  }
  if (pendingGstLen) {
    bleSend(pendingGst, pendingGstLen);
    pendingGstLen = 0;
  }
}

void sendTiltBle(unsigned long now) {
  if (!TILT_ENABLED || !tiltFresh || !deviceConnected) return;
  if (lastImuMs == 0 || now - lastImuMs > 3000) return;
  if (lastTiltBleMs != 0 && now - lastTiltBleMs < kTiltBleGapMs) return;
  if (bleCongested) {
    serviceBleCongestion(now);
    if (bleCongested) return;
  }
  if (lastTiltAttemptMs != 0 && now - lastTiltAttemptMs < kTiltBleRetryMs) return;
  const bool moved = fabsf(shownPitch - lastTiltSentPitch) >= kTiltShowEpsDeg ||
                     fabsf(shownRoll - lastTiltSentRoll) >= kTiltShowEpsDeg;
  const bool heartbeat = lastTiltBleMs == 0 || now - lastTiltBleMs >= 500;
  if (!moved && !heartbeat) return;
  lastTiltAttemptMs = now;
  // Binary on the dedicated char; ASCII on TX only as rare fallback (≤1 Hz).
  const bool binOk = bleSendTiltBinary(shownPitch, shownRoll);
  bool asciiOk = false;
  if (!binOk || lastTiltAsciiMs == 0 || now - lastTiltAsciiMs >= 1000) {
    char line[32];
    snprintf(line, sizeof(line), "$TILT,%.1f,%.1f\n", shownPitch, shownRoll);
    asciiOk = bleSend(line, strlen(line));
    if (asciiOk) lastTiltAsciiMs = now;
  }
  if (!binOk && !asciiOk) {
    tiltDrop++;
    return;
  }
  lastTiltSentPitch = shownPitch;
  lastTiltSentRoll = shownRoll;
  lastTiltBleMs = now;
  tiltTx++;
}

void sppSend(const uint8_t *data, size_t len) {
  if (!SerialBT.hasClient() || len == 0 || sppCongested) return;
  SerialBT.write(data, len);
}

void onSppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    sppUpEvent = true;
    sppCongested = false;
  } else if (event == ESP_SPP_CLOSE_EVT) {
    sppDownEvent = true;
    sppCongested = false;
  } else if (event == ESP_SPP_CONG_EVT) {
    sppCongested = param->cong.cong;
  }
}

// Survey Master writes RTCM and the occasional text command on the same classic socket.
// Stream complete RTCM frames into rtcmBuffer without capping at 520 bytes (BeiDou MSM can exceed that).
void drainSpp() {
  static uint8_t frame[3 + 1023 + 3];
  static size_t frameLen = 0;
  static size_t frameNeed = 0;
  static char line[128];
  static size_t lineLen = 0;

  int budget = 1024;
  while (budget-- > 0 && SerialBT.available() > 0) {
    int raw = SerialBT.read();
    if (raw < 0) break;
    uint8_t b = (uint8_t)raw;
    sppBytes++;
    if (frameLen > 0) {
      frame[frameLen++] = b;
      if (frameLen == 2 && (b & 0xFC)) {
        rtcmPush(frame, frameLen);
        frameLen = 0;
        continue;
      }
      if (frameLen == 3) frameNeed = 3 + ((((size_t)frame[1] & 0x03) << 8) | frame[2]) + 3;
      if (frameLen >= 3 && frameLen == frameNeed) {
        rtcmPush(frame, frameLen);
        frameLen = 0;
      }
      continue;
    }
    if (b == 0xD3 && lineLen == 0) {
      frame[0] = b;
      frameLen = 1;
      frameNeed = 0;
      continue;
    }
    if (b == '\r' || b == '\n') {
      if (lineLen == 0) continue;
      line[lineLen] = '\0';
      if (line[0] == '$') {
        handlePhoneCommand(line);
      } else if (!gnss.sendCommand(line)) {
        Serial.println("K902 command queue full");
      }
      lineLen = 0;
      continue;
    }
    if (b >= 0x20 && b <= 0x7E) {
      if (lineLen < sizeof(line) - 1) line[lineLen++] = b;
      else lineLen = 0;
      continue;
    }
    if (lineLen) {
      rtcmPush(line, lineLen);
      lineLen = 0;
    }
    rtcmPush(&b, 1);
  }
}

unsigned long lastGsvBleMs = 0;

void onGnssSentence(const char *sentence, size_t len) {
  if (rawEcho) Serial.write(sentence, len);
  sppSend(reinterpret_cast<const uint8_t *>(sentence), len);
  bool gga = sentenceType(sentence, len, "GGA,");
  bool rmc = sentenceType(sentence, len, "RMC,");
  bool gsa = sentenceType(sentence, len, "GSA,");
  bool gst = sentenceType(sentence, len, "GST,");
  bool gsv = sentenceType(sentence, len, "GSV,") && gsvFirstPage(sentence, len);
  if (!deviceConnected) {
    if (gsa) holdLine(pendingGsa, sizeof(pendingGsa), &pendingGsaLen, sentence, len);
    else if (gst) holdLine(pendingGst, sizeof(pendingGst), &pendingGstLen, sentence, len);
    return;
  }
  if (BLE_SEND_NMEA) {
    unsigned long t = millis();
    if (gga) {
      if (lastGgaBleMs != 0 && t - lastGgaBleMs < 1000) return;
      lastGgaBleMs = t;
    }
    // GSV floods 4 talkers × pages; 2 Hz is enough for the sky plot and frees BLE for $TILT.
    if (gsv) {
      if (lastGsvBleMs != 0 && t - lastGsvBleMs < 500) return;
      lastGsvBleMs = t;
    }
    // Skip RMC on BLE when GGA already carries the fix — same characteristic as the bubble.
    if (rmc) return;
    if (gga || gsa || gst || gsv) bleSend(sentence, len);
    return;
  }
  if (gsa || gst || gsv) bleSend(sentence, len);
}

void onGnssText(const char *line) {
  Serial.printf("K902> %s\n", line);
}

float gravityNorm() {
  return sqrtf(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
}

void bubbleLean(float *pitchDeg, float *rollDeg);

// Static tilt only (like a bubble level): the angle between measured gravity and the reference
// taken with the pole vertical. It is not valid while the pole is being swung.
void onGnssImu(const K902Imu &imu) {
  float raw = 0;
  for (int i = 0; i < 3; i++) raw += (float)(imu.accel[i] * imu.accel[i]);
  raw = sqrtf(raw);
  // The manual says m/s^2. A magnitude near 1 means this firmware reports g instead.
  float scale = (raw > 0.5f && raw < 2.5f) ? 9.80665f : 1.0f;
  static bool announced = false;
  if (!announced) {
    announced = true;
    Serial.printf("IMU id=%u |a|=%.3f %s acc=%.3f %.3f %.3f\n", imu.messageId, raw,
                  scale == 1.0f ? "m/s^2" : "g", imu.accel[0], imu.accel[1], imu.accel[2]);
  }
  float sample[3];
  for (int i = 0; i < 3; i++) {
    sample[i] = (float)imu.accel[i] * scale;
    if (!isfinite(sample[i])) return;
  }
  for (int i = 0; i < 3; i++) {
    gravity[i] = gravitySet ? gravity[i] + kTiltAccelBlend * (sample[i] - gravity[i]) : sample[i];
    if (!isfinite(gravity[i])) {
      gravitySet = false;
      return;
    }
  }
  gravitySet = true;
  lastImuMs = millis();
  float g = gravityNorm();
  if (g < 1.0f) return;
  float c = (gravity[0] * levelRef[0] + gravity[1] * levelRef[1] + gravity[2] * levelRef[2]) / g;
  tiltDeg = acosf(constrain(c, -1.0f, 1.0f)) * RAD_TO_DEG;
  float pitch = 0, roll = 0;
  bubbleLean(&pitch, &roll);
  if (!isfinite(pitch) || !isfinite(roll)) return;
  if (!tiltFresh || fabsf(pitch - shownPitch) >= kTiltShowEpsDeg || fabsf(roll - shownRoll) >= kTiltShowEpsDeg) {
    shownPitch = pitch;
    shownRoll = roll;
  }
  tiltFresh = true;
}

bool setLevelReference() {
  float g = gravityNorm();
  if (!gravitySet || millis() - lastImuMs > 1000 || g < 1.0f) {
    Serial.println("No IMU data from the K902 - tilt reference not changed");
    return false;
  }
  for (int i = 0; i < 3; i++) levelRef[i] = gravity[i] / g;
  prefs.putBytes("levelRef", levelRef, sizeof(levelRef));
  Serial.printf("Tilt reference saved: %.3f %.3f %.3f\n", levelRef[0], levelRef[1], levelRef[2]);
  return true;
}

// Lean of the pole away from the saved vertical, in the IMU plane. Both are 0 when the bubble is centered.
void bubbleLean(float *pitchDeg, float *rollDeg) {
  float g = gravityNorm();
  float ux = gravity[0] / g;
  float uy = gravity[1] / g;
  float uz = gravity[2] / g;
  float vx = levelRef[0], vy = levelRef[1], vz = levelRef[2];
  float gz = ux * vx + uy * vy + uz * vz;
  float ax = 0, ay = 0, az = 1;
  if (fabsf(vx * ax + vy * ay + vz * az) > 0.9f) {
    ax = 1;
    ay = 0;
    az = 0;
  }
  float cx = ay * vz - az * vy;
  float cy = az * vx - ax * vz;
  float cz = ax * vy - ay * vx;
  float cn = sqrtf(cx * cx + cy * cy + cz * cz);
  if (cn < 1e-4f) {
    *pitchDeg = 0;
    *rollDeg = 0;
    return;
  }
  cx /= cn;
  cy /= cn;
  cz /= cn;
  float yx = vy * cz - vz * cy;
  float yy = vz * cx - vx * cz;
  float yz = vx * cy - vy * cx;
  float gx = ux * cx + uy * cy + uz * cz;
  float gy = ux * yx + uy * yy + uz * yz;
  *rollDeg = atan2f(gx, gz) * RAD_TO_DEG;
  *pitchDeg = atan2f(gy, gz) * RAD_TO_DEG;
}

void sendPrtk() {
  // Only trust NMEA. IMU binary alone must not keep a stale FIXED badge on the phone.
  bool flowing = gnss.nmeaFlowing();
  const K902Fix &fix = gnss.fix();
  float hrms = 0;
  float vrms = 0;
  if (flowing && fix.sigmaLat >= 0 && fix.sigmaLon >= 0) {
    hrms = sqrtf(fix.sigmaLat * fix.sigmaLat + fix.sigmaLon * fix.sigmaLon);
  }
  if (flowing && fix.sigmaAlt >= 0) vrms = fix.sigmaAlt;
  char line[96];
  // Trailing age (GGA field 13) so the phone badge can show "รอ FLOAT" while quality is still SINGLE.
  if (flowing && fix.diffAge >= 0) {
    snprintf(line, sizeof(line), "$PRTK,%s,%u,%.2f,%.2f,%.3f,%.3f,%.0f\n",
             k902QualityName(fix.quality), fix.satellites,
             fix.pdop > 0 ? fix.pdop : 0.0f, fix.hdop, hrms, vrms, fix.diffAge);
  } else {
    snprintf(line, sizeof(line), "$PRTK,%s,%u,%.2f,%.2f,%.3f,%.3f\n",
             flowing ? k902QualityName(fix.quality) : "NODATA", flowing ? fix.satellites : 0,
             flowing && fix.pdop > 0 ? fix.pdop : 0.0f, flowing ? fix.hdop : 0.0f, hrms, vrms);
  }
  bleSend(line, strlen(line));
}

// Compact WGS84 UTM → lat/lon for $BCMD,START with N/E from the phone.
bool utmToLatLon(double easting, double northing, int zone, bool northern, double *lat, double *lon) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double k0 = 0.9996;
  const double e2 = f * (2.0 - f);
  const double ep2 = e2 / (1.0 - e2);
  double x = easting - 500000.0;
  double y = northern ? northing : northing - 10000000.0;
  double m = y / k0;
  double mu = m / (a * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0));
  double e1 = (1.0 - sqrt(1.0 - e2)) / (1.0 + sqrt(1.0 - e2));
  double phi1 = mu + (3.0 * e1 / 2.0 - 27.0 * e1 * e1 * e1 / 32.0) * sin(2.0 * mu)
              + (21.0 * e1 * e1 / 16.0 - 55.0 * e1 * e1 * e1 * e1 / 32.0) * sin(4.0 * mu)
              + (151.0 * e1 * e1 * e1 / 96.0) * sin(6.0 * mu);
  double sinP = sin(phi1), cosP = cos(phi1), tanP = tan(phi1);
  double n = a / sqrt(1.0 - e2 * sinP * sinP);
  double t = tanP * tanP;
  double c = ep2 * cosP * cosP;
  double r = a * (1.0 - e2) / pow(1.0 - e2 * sinP * sinP, 1.5);
  double d = x / (n * k0);
  double latRad = phi1 - (n * tanP / r) * (d * d / 2.0
      - (5.0 + 3.0 * t + 10.0 * c - 4.0 * c * c - 9.0 * ep2) * d * d * d * d / 24.0
      + (61.0 + 90.0 * t + 298.0 * c + 45.0 * t * t - 252.0 * ep2 - 3.0 * c * c) * d * d * d * d * d * d / 720.0);
  double lon0 = ((zone - 1) * 6 - 180 + 3) * DEG_TO_RAD;
  double lonRad = lon0 + (d - (1.0 + 2.0 * t + c) * d * d * d / 6.0
      + (5.0 - 2.0 * c + 28.0 * t - 3.0 * c * c + 8.0 * ep2 + 24.0 * t * t) * d * d * d * d * d / 120.0) / cosP;
  *lat = latRad * RAD_TO_DEG;
  *lon = lonRad * RAD_TO_DEG;
  return isfinite(*lat) && isfinite(*lon) && *lat >= -80.0 && *lat <= 84.0;
}

int zoneFromLon(double lon) {
  int z = (int)floor((lon + 180.0) / 6.0) + 1;
  if (z < 1) z = 1;
  if (z > 60) z = 60;
  return z;
}

void sendBstat() {
  const K902Fix &fix = gnss.fix();
  float hrms = 0;
  if (fix.sigmaLat >= 0 && fix.sigmaLon >= 0) {
    hrms = sqrtf(fix.sigmaLat * fix.sigmaLat + fix.sigmaLon * fix.sigmaLon);
  }
  char line[48];
  snprintf(line, sizeof(line), "$BSTAT,%s,%u,%.3f\n",
           baseActive ? "ON" : "OFF", fix.satellites, hrms);
  bleSend(line, strlen(line));
}

void onRtcmFromBoard(const uint8_t *data, size_t len) {
  if ((!baseActive && !rawLogActive) || len == 0) return;
  lastRtcmOutMs = millis();
  if (deviceConnected) queueRtcmToPhone(data, len);
}

// Raw observations for RINEX: MSM4 per constellation plus GPS/GLONASS ephemerides.
// Galileo and BeiDou ephemerides come from an IGS broadcast file at conversion time.
struct RawLog {
  const char *name;
  uint8_t seconds;
};
const RawLog kRawLogs[] = {
  {"rtcm1074b", 1}, {"rtcm1084b", 1}, {"rtcm1094b", 1}, {"rtcm1124b", 1},
  {"rtcm1019b", 30}, {"rtcm1020b", 30},
};
const size_t kRawLogCount = sizeof(kRawLogs) / sizeof(kRawLogs[0]);

bool armRawLog() {
  lastRawArmMs = millis();
  if (gnss.freeCommandSlots() < kRawLogCount) {
    Serial.println("Raw log: K902 command queue busy");
    return false;
  }
  char cmd[64];
  for (size_t i = 0; i < kRawLogCount; i++) {
    snprintf(cmd, sizeof(cmd), "log " K902_PORT " %s ontime %u", kRawLogs[i].name, kRawLogs[i].seconds);
    gnss.sendCommand(cmd);
  }
  return true;
}

void disarmRawLog() {
  char cmd[64];
  for (size_t i = 0; i < kRawLogCount; i++) {
    snprintf(cmd, sizeof(cmd), "unlog " K902_PORT " %s", kRawLogs[i].name);
    if (!gnss.sendCommand(cmd)) Serial.printf("K902 queue full at %s\n", cmd);
  }
}

// "$LSTAT,ON,<frames queued>,<frames dropped for a full queue>,<notifications the BLE stack failed>"
void sendLstat() {
  char line[64];
  snprintf(line, sizeof(line), "$LSTAT,%s,%lu,%lu,%lu\n", rawLogActive ? "ON" : "OFF",
           (unsigned long)rawFramesQueued, (unsigned long)rawFramesDropped, (unsigned long)bleNotifyFails);
  bleSend(line, strlen(line));
  lastLstatMs = millis();
}

const char *const kBaseCommands[] = {
  "unlogall " K902_PORT,
  "log " K902_PORT " rtcm1074b ontime 1",
  "log " K902_PORT " rtcm1084b ontime 1",
  "log " K902_PORT " rtcm1094b ontime 1",
  "log " K902_PORT " rtcm1124b ontime 1",
  "log " K902_PORT " rtcm1005b ontime 5",
  "log " K902_PORT " rtcm1033b ontime 10",
  "log gpgga ontime 1",
  "log " K902_PORT " imudatab ontime 0.1",  // keep bubble alive while base is transmitting
};

// zone <= 0 means the phone did not send one; fall back to the zone of the live fix.
bool startBaseFromUtm(double north, double east, double elev, double antH, int zone, bool northern) {
  const K902Fix &fix = gnss.fix();
  if (zone <= 0) {
    if (!fix.hasPosition()) {
      Serial.println("BCMD has no zone and the K902 has no fix");
      return false;
    }
    zone = zoneFromLon(fix.longitude);
    northern = fix.latitude >= 0;
  }
  if (zone > 60) return false;
  double lat = 0, lon = 0;
  if (!utmToLatLon(east, north, zone, northern, &lat, &lon)) {
    Serial.println("BCMD UTM convert failed");
    return false;
  }
  const size_t needed = sizeof(kBaseCommands) / sizeof(kBaseCommands[0]) + 1;
  if (gnss.freeCommandSlots() < needed) {
    Serial.println("BCMD K902 command queue busy");
    return false;
  }
  double arp = elev + antH;
  char fixCmd[96];
  snprintf(fixCmd, sizeof(fixCmd), "fix position %.9f %.9f %.4f", lat, lon, arp);
  gnss.sendCommand(kBaseCommands[0]);
  gnss.sendCommand(fixCmd);
  for (size_t i = 1; i < sizeof(kBaseCommands) / sizeof(kBaseCommands[0]); i++) {
    gnss.sendCommand(kBaseCommands[i]);
  }
  if (rtcmMux && xSemaphoreTake(rtcmMux, pdMS_TO_TICKS(20)) == pdTRUE) {
    xStreamBufferReset(rtcmBuffer);
    xSemaphoreGive(rtcmMux);
  }
  baseActive = true;
  Serial.printf("Base ON fix %.9f %.9f %.3f zone %d%c\n", lat, lon, arp, zone, northern ? 'N' : 'S');
  return true;
}

void stopBase() {
  // Must land unlogall + fix none or the K902 stays on a monument and rover coords freeze.
  if (gnss.freeCommandSlots() < 2) {
    Serial.println("Base stop deferred - K902 command queue full");
    return;
  }
  if (!gnss.sendCommand("unlogall " K902_PORT) || !gnss.sendCommand("fix none")) {
    Serial.println("Base stop failed to queue fix none - will retry");
    return;
  }
  // Re-arm rover logs even if NMEA is still flowing (configure() would no-op then).
  for (size_t i = 0; i < sizeof(kK902Setup) / sizeof(kK902Setup[0]); i++) {
    if (strncmp(kK902Setup[i], "unlogall", 8) == 0 || strcmp(kK902Setup[i], "fix none") == 0) continue;
    if (!gnss.sendCommand(kK902Setup[i])) Serial.printf("K902 queue full at %s\n", kK902Setup[i]);
  }
  baseActive = false;
  // If the queue is too full now, the re-arm check in loop() retries in 20 s.
  if (rawLogActive) armRawLog();
  Serial.println("Base OFF");
}

void sendBstatOff() {
  const char *line = "$BSTAT,OFF,0,0\n";
  bleSend(line, strlen(line));
}

void handlePhoneCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) {
    if (*p == '\r' || *p == '\n') *p = '\0';
  }
  Serial.printf("Phone< %s\n", cmd);
  if (strncmp(cmd, "$ICMD,CALIB", 11) == 0) {
    const char *reply = setLevelReference() ? "$ICALIB,OK\n" : "$ICALIB,FAIL,no_imu\n";
    bleSend(reply, strlen(reply));
    Serial.printf("Phone> %s", reply);
    return;
  }
  if (strncmp(cmd, "$ICMD,HEIGHT", 12) == 0) {
    // The bubble uses the saved level reference; the pole height only matters on the phone.
    return;
  }
  if (strncmp(cmd, "$LCMD,START", 11) == 0) {
    // In base mode the K902 already sends MSM4, so only the forwarding switch is needed.
    if (!rawLogActive && !baseActive && !armRawLog()) {
      sendLstat();
      return;
    }
    if (!rawLogActive) {
      rawFramesQueued = 0;
      rawFramesDropped = 0;
      bleNotifyFails = 0;
    }
    rawLogActive = true;
    Serial.println("Raw log ON");
    sendLstat();
    return;
  }
  if (strncmp(cmd, "$LCMD,STOP", 10) == 0) {
    if (rawLogActive && !baseActive) disarmRawLog();
    rawLogActive = false;
    Serial.println("Raw log OFF");
    sendLstat();
    return;
  }
  if (strncmp(cmd, "$BCMD,STOP", 10) == 0) {
    if (baseActive) stopBase();
    sendBstat();
    return;
  }
  if (strncmp(cmd, "$BCMD,START,", 12) == 0) {
    double north = 0, east = 0, elev = 0, antH = 0;
    int zone = 0;
    char hemi = 'N';
    int got = sscanf(cmd + 12, "%lf,%lf,%lf,%lf,%d,%c", &north, &east, &elev, &antH, &zone, &hemi);
    if (got != 4 && got != 6) {
      Serial.println("BCMD parse fail");
      sendBstatOff();
      return;
    }
    if (got == 4) zone = 0;
    if (!startBaseFromUtm(north, east, elev, antH, zone, hemi != 'S' && hemi != 's')) {
      sendBstatOff();
      return;
    }
    sendBstat();
    return;
  }
}

void printStatus() {
  if (!gnss.dataFlowing()) {
    Serial.printf("[K902] no NMEA at %lu baud - check K902 TXD->GPIO%d, RXD<-GPIO%d, GND, power\n",
                  (unsigned long)gnss.baud(), K902_RX_PIN, K902_TX_PIN);
    return;
  }
  const K902Fix &fix = gnss.fix();
  Serial.printf("[K902] %s sats=%u hdop=%.1f", k902QualityName(fix.quality), fix.satellites, fix.hdop);
  if (fix.hasPosition()) {
    Serial.printf(" lat=%.9f lon=%.9f alt=%.3f", fix.latitude, fix.longitude, fix.altitude);
  }
  if (fix.sigmaLat >= 0) Serial.printf(" sd=%.3f/%.3f/%.3fm", fix.sigmaLat, fix.sigmaLon, fix.sigmaAlt);
  if (fix.diffAge >= 0) Serial.printf(" age=%.0fs", fix.diffAge);
  if (TILT_ENABLED && millis() - lastImuMs < 1000) {
    Serial.printf(" tilt=%.1fdeg |a|=%.2f id=%u", tiltDeg, gravityNorm(), gnss.imuMessageId());
  } else if (TILT_ENABLED && gnss.otherBinaryId() != 0) {
    Serial.printf(" tilt=-- binary id=%u len=%u", gnss.otherBinaryId(), gnss.otherBinaryLength());
  } else if (TILT_ENABLED) {
    Serial.print(" tilt=-- (no IMUDATA)");
  }
  if (rawLogActive || baseActive) {
    Serial.printf(" | up RTCM %lu frames, %lu dropped, queue %u, BLE fails %lu%s", (unsigned long)rawFramesQueued,
                  (unsigned long)rawFramesDropped, (unsigned)rawCount, (unsigned long)bleNotifyFails,
                  bleCongested ? " CONGESTED" : "");
  }
  Serial.printf(" | BLE %s %s q=%u, SPP %s %lu B, phone %lu B/%lu writes, wait %u, RTCM %lu B/%lu frames base=%lu gps=%lu glo=%lu gal=%lu bds=%lu qzs=%lu last=%u tilt %lu/%lu\n",
                deviceConnected ? "connected" : "advertising", gnss.portState(), gnss.pendingCommands(),
                SerialBT.hasClient() ? "connected" : "waiting", (unsigned long)sppBytes,
                (unsigned long)bleWriteBytes, (unsigned long)bleWriteCount,
                (unsigned)xStreamBufferBytesAvailable(rtcmBuffer),
                (unsigned long)gnss.correctionBytes(), (unsigned long)gnss.correctionFrames(),
                (unsigned long)gnss.correctionBasePos(), (unsigned long)gnss.correctionGps(),
                (unsigned long)gnss.correctionGlo(), (unsigned long)gnss.correctionGal(),
                (unsigned long)gnss.correctionBds(), (unsigned long)gnss.correctionQzs(),
                (unsigned)gnss.lastCorrectionType(), tiltTx, tiltDrop);
  if (phoneFirstWritePending) {
    phoneFirstWritePending = false;
    Serial.printf("Phone RX first write %u B (0x%02X) gatts_h=%u seen=%lu\n",
                  (unsigned)lastGattsWriteLen, (unsigned)lastGattsWriteByte,
                  (unsigned)lastGattsWriteHandle, (unsigned long)gattsWriteSeen);
  }
  if (deviceConnected && bleWriteCount == 0 && !phoneWriteWarned &&
      millis() - bleConnectedAtMs >= 8000) {
    phoneWriteWarned = true;
    Serial.println("WARN: BLE up 8s but phone wrote 0 B to RX — reinstall app, forget BLE, reconnect, start NTRIP");
  }
}

// Serial monitor: type a line and press Enter. "!quiet" stops the [K902] flood so you can type.
// "!raw" toggles NMEA echo, "!init" re-sends setup, "!level" stores vertical, anything else
// goes to the K902 (e.g. "log version", "log imudataa ontime 1").
void handleConsole() {
  static char line[128];
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c != '\r' && c != '\n') {
      if (consoleLen < sizeof(line) - 1) line[consoleLen++] = c;
      continue;
    }
    if (consoleLen == 0) continue;
    line[consoleLen] = '\0';
    consoleLen = 0;
    if (strcmp(line, "!raw") == 0) {
      rawEcho = !rawEcho;
      Serial.printf("NMEA echo %s\n", rawEcho ? "on" : "off");
    } else if (strcmp(line, "!quiet") == 0) {
      quietStatus = !quietStatus;
      Serial.printf("Status print %s\n", quietStatus ? "off" : "on");
    } else if (strcmp(line, "!init") == 0) {
      gnss.configure();
    } else if (strcmp(line, "!level") == 0) {
      setLevelReference();
    } else if (gnss.sendCommand(line)) {
      Serial.printf("K902< %s\n", line);
    } else {
      Serial.println("K902 command queue full");
    }
  }
}

const char *disconnectHint(uint16_t reason) {
  switch (reason) {
    case 0x05:
    case 0x06: return "pairing keys mismatch - forget this device in the phone's Bluetooth settings";
    case 0x08: return "link timeout - weak signal or unstable power";
    case 0x13: return "closed by the phone";
    case 0x16: return "closed by the ESP32";
    case 0x3E: return "link could not be established - weak signal or unstable power";
    default: return "";
  }
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "CRASH";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT - supply voltage dropped, check power";
    default: return "other";
  }
}

void printLinkParams() {
  Serial.printf("interval %.2f ms, latency %u, timeout %.2f s\n", connInterval * 1.25f, connLatency,
                connTimeout / 100.0f);
}

void serviceBle(unsigned long now) {
  if (connectEvent) {
    connectEvent = false;
    oldDeviceConnected = true;
    stepBaseSet = false;
    paramsRequested = false;
    connectedAt = now;
    lastTiltBleMs = 0;
    Serial.printf("Bluetooth Connected! %02X:%02X:%02X:%02X:%02X:%02X, ", peerAddr[0], peerAddr[1],
                  peerAddr[2], peerAddr[3], peerAddr[4], peerAddr[5]);
    printLinkParams();
  }
  if (paramsEvent) {
    paramsEvent = false;
    if (paramsRejected) {
      paramsRejected = false;
      Serial.println("Bluetooth link update rejected by the phone");
    } else {
      Serial.print("Bluetooth link updated: ");
      printLinkParams();
    }
  }
  if (mtuEvent) {
    mtuEvent = false;
    Serial.printf("Bluetooth MTU %u\n", peerMtu);
  }
  if (authEvent) {
    authEvent = false;
    if (authOk) {
      Serial.println("Bluetooth pairing OK");
    } else {
      Serial.printf("Bluetooth pairing FAILED reason 0x%02X - forget the device on the phone\n", authReason);
    }
  }
  // A short supervision timeout drops the link whenever a body blocks the signal for a moment.
  if (oldDeviceConnected && !paramsRequested && now - connectedAt >= 1000) {
    paramsRequested = true;
    // Prefer ~7.5–15 ms connection events for the tilt characteristic.
    if (connTimeout < 400) pServer->updateConnParams(peerAddr, 6, 12, 0, 600);
  }
  if (advFailedEvent) {
    advFailedEvent = false;
    if (!deviceConnected) {
      Serial.println("Bluetooth advertising failed to start - retrying");
      advertisePending = true;
      disconnectedAt = now;
    }
  }
  if (disconnectEvent) {
    disconnectEvent = false;
    oldDeviceConnected = false;
    advertisePending = true;
    disconnectedAt = now;
    clearRawQueue();
    phoneLostAt = now;
    uint16_t reason = disconnectReason;
    Serial.printf("Bluetooth Disconnected! reason 0x%02X %s\n", reason, disconnectHint(reason));
  }
  if (advertisePending && now - disconnectedAt >= 400) {
    advertisePending = false;
    pServer->startAdvertising();
    Serial.println("Restart advertising");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("\nReset reason: %s\n", resetReasonName(esp_reset_reason()));
  rtcmMux = xSemaphoreCreateMutex();
  phoneCmdQueue = xQueueCreate(4, kPhoneCmdMax);
  rtcmBuffer = xStreamBufferCreate(4096, 1);
  prefs.begin("rover");
  if (prefs.isKey("levelRef")) {
    prefs.getBytes("levelRef", levelRef, sizeof(levelRef));
    float n = sqrtf(levelRef[0] * levelRef[0] + levelRef[1] * levelRef[1] + levelRef[2] * levelRef[2]);
    if (!isfinite(n) || n < 0.5f || n > 1.5f) {
      levelRef[0] = 0;
      levelRef[1] = 0;
      levelRef[2] = 1;
      Serial.println("levelRef from NVS invalid - reset to +Z");
    } else {
      levelRef[0] /= n;
      levelRef[1] /= n;
      levelRef[2] /= n;
    }
  }

  gnss.onSentence(onGnssSentence);
  gnss.onFix(onGnssFix);
  gnss.onText(onGnssText);
  gnss.onImu(onGnssImu);
  gnss.onRtcmOut(onRtcmFromBoard);
  gnss.begin(Serial2, K902_RX_PIN, K902_TX_PIN, K902_BAUD, kK902Setup,
             sizeof(kK902Setup) / sizeof(kK902Setup[0]), K902_RTCM_INPUT);

  // Classic SPP first so the controller starts in dual mode, then BLE for this app.
  SerialBT.register_callback(onSppEvent);
  if (SerialBT.begin(BLE_DEVICE_NAME)) {
    Serial.println("Classic Bluetooth SPP ready (Survey Master)");
  } else {
    Serial.println("Classic Bluetooth SPP failed");
  }

  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247);
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);
  BLEDevice::setCustomGapHandler(onGap);
  BLEDevice::setCustomGattsHandler(onGatts);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  pTxCharacteristic->setCallbacks(new TxCallbacks());

  pTiltCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TILT,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pTiltCharacteristic->addDescriptor(new BLE2902());
  pTiltCharacteristic->setCallbacks(new TxCallbacks());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  rxCharHandle = pRxCharacteristic->getHandle();
  Serial.printf("RX char %s handle=%u (WRITE + WRITE_NR)\n",
                pRxCharacteristic->getUUID().toString().c_str(), (unsigned)rxCharHandle);
  BLEAdvertising *adv = pServer->getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();
  Serial.println("Waiting for phone...");
}

void loop() {
  unsigned long now = millis();
  serviceBleCongestion(now);
  serviceBle(now);
  if (sppUpEvent) {
    sppUpEvent = false;
    Serial.println("Survey Master Bluetooth connected");
  }
  if (sppDownEvent) {
    sppDownEvent = false;
    Serial.println("Survey Master Bluetooth disconnected");
  }
  drainSpp();
  handleConsole();
  char cmd[kPhoneCmdMax];
  while (phoneCmdQueue && xQueueReceive(phoneCmdQueue, cmd, 0) == pdTRUE) {
    cmd[kPhoneCmdMax - 1] = '\0';
    handlePhoneCommand(cmd);
  }
  // The phone is the caster. With it gone this long nobody gets the corrections, and a
  // forgotten fixed position would freeze the next rover session.
  if (!oldDeviceConnected && now - phoneLostAt >= 60000) {
    if (baseActive) {
      Serial.println("Phone gone 60 s - stopping base");
      stopBase();
    }
    if (rawLogActive) {
      Serial.println("Phone gone 60 s - stopping raw log");
      disarmRawLog();
      rawLogActive = false;
    }
  }
  // A K902 power cycle drops its logs; ask again when observations stop while NMEA still runs.
  if (rawLogActive && !baseActive && gnss.nmeaFlowing() && now - lastRtcmOutMs > 15000 &&
      now - lastRawArmMs > 20000) {
    Serial.println("Raw log: no RTCM from K902 - re-arming");
    armRawLog();
  }

  gnss.poll();

  if (!baseActive && gnss.acceptsCorrections()) {
    uint8_t rtcm[256];
    size_t n;
    while ((n = xStreamBufferReceive(rtcmBuffer, rtcm, sizeof(rtcm), 0)) > 0) {
      if (gnss.writeCorrections(rtcm, n) == 0) break;
    }
  }
  sendTiltBle(now);   // bubble first — same TX char as raw RTCM / NMEA
  serviceRawTx(now);
  flushQualityLines();

  if (now - lastStatus >= 1000) {
    lastStatus = now;
    if (!quietStatus && consoleLen == 0) printStatus();
    if (BLE_SEND_PRTK_S && !baseActive) sendPrtk();
    if (baseActive && now - lastBstatMs >= 1000) {
      lastBstatMs = now;
      sendBstat();
    }
    if (rawLogActive && now - lastLstatMs >= 5000) sendLstat();
  }
  delay(1);
}
