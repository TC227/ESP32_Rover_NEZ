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
#include <Update.h>
#include <esp_ota_ops.h>

#include "K902.h"
#include "Hc12Radio.h"

// ESP32 UART2 <-> K902 UART2 (COM2), 3.3 V TTL (never wire its RS-232 DB9 directly):
// K902 TXD -> GPIO16 (RX2), K902 RXD <- GPIO17 (TX2), GND <-> GND.
#define K902_RX_PIN 16
#define K902_TX_PIN 17
#define K902_BAUD 115200
// HC-12 433 MHz (same on base and rover): TXD->GPIO26, RXD<-GPIO27, SET->GPIO25, VCC 5V, GND.
#define HC12_RX_PIN 26
#define HC12_TX_PIN 27
#define HC12_SET_PIN 25
#define BUZZER_PIN 23  // active buzzer I/O, high = beep. VCC from 5V on the carrier.
#define K902_PORT "COM2"   // K902 port those wires go to
#define K902_RTCM_INPUT 1  // RTCM3 written by the phone to the RX characteristic is fed to the K902

#define BLE_DEVICE_NAME "E25aer_gnss"  // one binary; app sends $ROLE,BASE|ROVER after connect
#define FW_VERSION "1.2.16"
// Build with -DFORCE_BASE_ROLE=1 (env esp32dev_base) for a head locked to BASE:
// $ROLE,ROVER is ignored, boot always starts as BASE, $FW reports the -BASE label.
#ifndef FORCE_BASE_ROLE
#define FORCE_BASE_ROLE 0
#endif
#if FORCE_BASE_ROLE
#define FW_LABEL "1.2.16-BASE"
#else
#define FW_LABEL FW_VERSION
#endif
#define BLE_SEND_NMEA 1    // every NMEA sentence on BLE; SW Maps needs GGA and RMC for a position
#define BLE_SEND_PRTK_S 1  // "$PRTK,<fix>,<sats>" 1 Hz + "$S,<seq>,<dN cm>,<dE cm>" movement per fix
#define TILT_ENABLED 1     // pole bubble from the K902 IMU: status line + "$TILT,<pitch>,<roll>"
#define K902_INS_ENABLED 1 // GNSS+INS + INSRTK tip survey when the board firmware supports it
// Bubble snappiness: higher blend / lower thresholds = follows the pole faster (more IMU noise).
// Accel-only bubbles feel bad while swinging (linear accel ≠ gravity). Gate hard when
// |a|≠g, and fuse gyro when the short IMUDATA frame carries it. BLE ~25 Hz binary tilt.
static const float kTiltAccelBlendStatic = 0.92f; // settle to gravity faster when held still
static const float kTiltAccelBlendDynamic = 0.12f;
static const float kTiltStaticGAbs = 1.8f;    // | |a|-g | below this → trust accel
static const float kTiltShowEpsDeg = 0.01f;
static const unsigned kTiltBleGapMs = 25;     // ~40 Hz bubble notify
static const unsigned kTiltBleRetryMs = 10;
static const float kTiltGyroAlpha = 0.72f;    // complementary: lower = return to level faster when still

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// Dedicated notify lane for the pole bubble — not shared with NMEA / RTCM on TX.
#define CHARACTERISTIC_UUID_TILT "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
// Firmware image only. NMEA, RTCM, and $ commands stay on RX/TX/TILT.
#define CHARACTERISTIC_UUID_OTA "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"
static const uint8_t kOtaStart = 0x01;
static const uint8_t kOtaChunk = 0x02;
static const uint8_t kOtaFinish = 0x03;
static const uint8_t kOtaAbort = 0x04;
// Binary tilt packet: [0x1F][pitch_cdeg int16 LE][roll_cdeg int16 LE]
static const uint8_t kTiltPacketTag = 0x1F;

// Sent to the K902 after boot and again whenever its output stops.
const char *const kK902Setup[] = {
  "unlogall " K902_PORT,
  "fix none",  // a base session left by a previous ESP32 run must not freeze the rover position
  "unlockoutall",
  "rtkobsmode 0",
  "set appscene survey",
#if K902_INS_ENABLED
  "inscontrol enable",
  "set smootheddr on",
  // imuaxestype is not sent here. The manual applies it only after saveconfig and a
  // reboot, and that reboot drops every command still queued, including imudatab.
  "set drleverarm 0 0 0",
  "set drconfig 1.0",    // walk-up init ~1 m/s; default vehicle threshold never aligns on a pole
  "set rtkfreq 10",
  "set pvtfreq 10",
#endif
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
  "log " K902_PORT " imudatab ontime 0.05",
#endif
#if K902_INS_ENABLED
  // Required for true tilted-pole surveying.  IMUDATA drives only the bubble;
  // INSRTKCOMPENSATE carries the K902-calculated pole-tip ENU offset.
  "log " K902_PORT " insrtkcompensateb ontime 0.2",
#endif
  // rawimub at 100 Hz fills the 115200 COM2 link and drops imudatab, so the
  // bubble and level calibration see no IMU. insconfigb is only on change.
  "unlog " K902_PORT " rawimub",
  "log " K902_PORT " insconfigb onchanged",
};

K902 gnss;
Hc12Radio hc12;
enum class HeadRole : uint8_t { Rover = 0, Base = 1 };
HeadRole headRole = HeadRole::Rover;
bool isBaseRole() { return headRole == HeadRole::Base; }
const char *headRoleName() { return isBaseRole() ? "BASE" : "ROVER"; }
unsigned long lastBaseLinkStatusMs = 0;
unsigned long lastRadioRtcmMs = 0;
enum class RoverCorrectionSource : uint8_t { Radio = 0, PhoneNtrip = 1 };
RoverCorrectionSource roverCorrectionSource = RoverCorrectionSource::Radio;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
BLECharacteristic *pTiltCharacteristic = NULL;
BLECharacteristic *pOtaCharacteristic = NULL;
bool otaActive = false;
uint32_t otaExpected = 0;
uint32_t otaWritten = 0;
volatile bool otaRebootPending = false;
unsigned long otaRebootAt = 0;
void abortOta();
volatile bool deviceConnected = false;
volatile unsigned long bleConnectedAtMs = 0;
volatile unsigned long lastBleTrafficMs = 0;
uint16_t peerConnId = 0;
volatile bool phoneWriteAnnounced = false;
volatile bool phoneWriteWarned = false;
volatile uint16_t peerMtu = 23;
volatile bool connectEvent = false;
volatile bool fwAnnouncePending = false;
unsigned long lastFwAnnounceMs = 0;
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
bool bleCutPending = false;
unsigned long bleCutAt = 0;
unsigned long lastRadioStatusMs = 0;
SemaphoreHandle_t hc12Mux = NULL;
unsigned long lastBstatMs = 0;
volatile bool rawLogActive = false;  // phone is recording the K902's own RTCM observations
unsigned long lastRtcmOutMs = 0;
unsigned long rtcmOutFrames = 0;   // complete RTCM3 frames received from the K902
unsigned long lastRawArmMs = 0;
unsigned long lastLstatMs = 0;
// Why the previous boot ended — survives until the next reset, so $PRTK can show it.
int lastResetReason = -1;

void handlePhoneCommand(char *cmd);
void bleStart();
void bleStop();

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
float fusedPitch = 0;           // complementary-filter lean (IMU frame)
float fusedRoll = 0;
bool fusedInit = false;
bool tiltFresh = false;
bool tiltGyroLive = false;
bool insRtkLive = false;
bool insInitPending = false;
int8_t lastInsStatus = -99;
float poleHeightM = 2.0f;
unsigned long lastImuMs = 0;
unsigned long lastImuRearmMs = 0;
unsigned long lastImuSampleMs = 0;
unsigned long lastPinsMs = 0;
unsigned long insInitStartedMs = 0;
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
    lastBleTrafficMs = bleConnectedAtMs;
    peerConnId = param->connect.conn_id;
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
    if (pOtaCharacteristic != NULL) {
      BLE2902 *otaCcc = (BLE2902 *)pOtaCharacteristic->getDescriptorByUUID((uint16_t)0x2902);
      if (otaCcc != NULL) otaCcc->setNotifications(true);
    }
  }
  void onDisconnect(BLEServer *server, esp_ble_gatts_cb_param_t *param) {
    if (otaActive) abortOta();
    deviceConnected = false;
    peerConnId = 0;
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
  static const char *const kPrefixes[] = {"$ICMD,", "$BCMD,", "$LCMD,", "$RCMD,", "$WCMD,", "$ROLE", "$FW", "$VER"};
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
    lastBleTrafficMs = millis();
    data += n;
    len -= n;
    sent += n;
  }
  return true;
}

void sppSend(const uint8_t *data, size_t len);

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
  if (!isBaseRole() && roverCorrectionSource == RoverCorrectionSource::PhoneNtrip) {
    rtcmPush(data, len);
  }
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
    lastBleTrafficMs = millis();
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
  if (!TILT_ENABLED || !tiltFresh) return;
  if (!deviceConnected) return;
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
  // Binary on the dedicated char. ASCII only if binary fails — never both (dual path = jitter).
  const bool binOk = bleSendTiltBinary(shownPitch, shownRoll);
  bool asciiOk = false;
  if (!binOk) {
    char line[32];
    snprintf(line, sizeof(line), "$TILT,%.2f,%.2f\n", shownPitch, shownRoll);
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

// Spirit-level attitude: trust accel only when nearly static; otherwise coast on gyro (or hold).
void onGnssImu(const K902Imu &imu) {
  float raw = 0;
  for (int i = 0; i < 3; i++) raw += (float)(imu.accel[i] * imu.accel[i]);
  raw = sqrtf(raw);
  // The manual says m/s^2. A magnitude near 1 means this firmware reports g instead.
  float scale = (raw > 0.5f && raw < 2.5f) ? 9.80665f : 1.0f;
  static bool announced = false;
  if (!announced) {
    announced = true;
    Serial.printf("IMU id=%u |a|=%.3f %s acc=%.3f %.3f %.3f gyro=%.4f %.4f %.4f\n",
                  imu.messageId, raw, scale == 1.0f ? "m/s^2" : "g",
                  imu.accel[0], imu.accel[1], imu.accel[2],
                  imu.gyro[0], imu.gyro[1], imu.gyro[2]);
  }
  float sample[3];
  for (int i = 0; i < 3; i++) {
    sample[i] = (float)imu.accel[i] * scale;
    if (!isfinite(sample[i])) return;
  }
  const float sampleG = sqrtf(sample[0] * sample[0] + sample[1] * sample[1] + sample[2] * sample[2]);
  const bool nearlyStatic = fabsf(sampleG - 9.80665f) < kTiltStaticGAbs;
  const float blend = nearlyStatic ? kTiltAccelBlendStatic : kTiltAccelBlendDynamic;
  for (int i = 0; i < 3; i++) {
    gravity[i] = gravitySet ? gravity[i] + blend * (sample[i] - gravity[i]) : sample[i];
    if (!isfinite(gravity[i])) {
      gravitySet = false;
      return;
    }
  }
  gravitySet = true;
  unsigned long now = millis();
  float dt = lastImuSampleMs ? (now - lastImuSampleMs) * 0.001f : 0.05f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.2f) dt = 0.2f;
  lastImuSampleMs = now;
  lastImuMs = now;

  float g = gravityNorm();
  if (g < 1.0f) return;
  float c = (gravity[0] * levelRef[0] + gravity[1] * levelRef[1] + gravity[2] * levelRef[2]) / g;
  tiltDeg = acosf(constrain(c, -1.0f, 1.0f)) * RAD_TO_DEG;
  float accelPitch = 0, accelRoll = 0;
  bubbleLean(&accelPitch, &accelRoll);
  if (!isfinite(accelPitch) || !isfinite(accelRoll)) return;

  // Gyro from full IMUDATA or short 48-byte (gyro||accel) layout. Units: rad/s preferred;
  // if magnitude looks like deg/s (>~10), convert.
  float gx = (float)imu.gyro[0], gy = (float)imu.gyro[1], gz = (float)imu.gyro[2];
  const float gmag = fabsf(gx) + fabsf(gy) + fabsf(gz);
  tiltGyroLive = gmag > 1e-6f;
  if (tiltGyroLive && gmag > 10.0f) {
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
  }

  if (!fusedInit) {
    fusedPitch = accelPitch;
    fusedRoll = accelRoll;
    fusedInit = true;
  } else if (tiltGyroLive && nearlyStatic) {
    // Map body rates onto lean axes used by bubbleLean (IMU pitch/roll of the pole).
    fusedPitch += gy * RAD_TO_DEG * dt;
    fusedRoll += gx * RAD_TO_DEG * dt;
    fusedPitch = kTiltGyroAlpha * fusedPitch + (1.0f - kTiltGyroAlpha) * accelPitch;
    fusedRoll = kTiltGyroAlpha * fusedRoll + (1.0f - kTiltGyroAlpha) * accelRoll;
  } else {
    // Always follow gravity. Holding the last angle while |a| leaves 1 g left the
    // bubble stuck on a rover whose first samples were not a clean static reading.
    const float follow = nearlyStatic ? 0.85f : 0.45f;
    fusedPitch += follow * (accelPitch - fusedPitch);
    fusedRoll += follow * (accelRoll - fusedRoll);
  }

  shownPitch = fusedPitch;
  shownRoll = fusedRoll;
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
  char line[256];
  int n;
  // Trailing age (GGA field 13) so the phone badge can show "รอ FLOAT" while quality is still SINGLE.
  if (flowing && fix.diffAge >= 0) {
    n = snprintf(line, sizeof(line), "$PRTK,%s,%u,%.2f,%.2f,%.3f,%.3f,%.0f",
             k902QualityName(fix.quality), fix.satellites,
             fix.pdop > 0 ? fix.pdop : 0.0f, fix.hdop, hrms, vrms, fix.diffAge);
  } else {
    n = snprintf(line, sizeof(line), "$PRTK,%s,%u,%.2f,%.2f,%.3f,%.3f",
             flowing ? k902QualityName(fix.quality) : "NODATA", flowing ? fix.satellites : 0,
             flowing && fix.pdop > 0 ? fix.pdop : 0.0f, flowing ? fix.hdop : 0.0f, hrms, vrms);
  }
  // Bubble diagnostics for the phone log: tilt tx/drop, IMU frames seen, IMU age, decode state.
  const unsigned long imuAge = lastImuMs == 0 ? 0xFFFFFFFFUL : (unsigned long)(millis() - lastImuMs);
  const unsigned long rtcmAge = lastRtcmOutMs == 0 ? 0xFFFFFFFFUL : (unsigned long)(millis() - lastRtcmOutMs);
  n += snprintf(line + n, sizeof(line) - (size_t)n, ",%lu,%lu,%lu,%lu,%d,%u,%u,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu",
             (unsigned long)tiltTx, (unsigned long)tiltDrop,
             (unsigned long)gnss.imuFrames(), imuAge, tiltFresh ? 1 : 0,
             (unsigned)gnss.otherBinaryId(), (unsigned)gnss.otherBinaryLength(),
             baseActive ? 1 : 0, hc12.atOk() ? 1 : 0, (unsigned long)hc12.baud(),
             (unsigned long)hc12.txBytes(), (unsigned long)hc12.rxBytes(),
             (unsigned long)hc12.drops(), rtcmOutFrames, rtcmAge);
  n += snprintf(line + n, sizeof(line) - (size_t)n, ",%d\n", lastResetReason);
  bleSend(line, (size_t)n);
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

bool hc12Lock(uint32_t waitMs) {
  if (!hc12Mux) return false;
  return xSemaphoreTake(hc12Mux, pdMS_TO_TICKS(waitMs)) == pdTRUE;
}

void hc12Unlock() {
  if (hc12Mux) xSemaphoreGive(hc12Mux);
}

// AT uses delay(). It runs here so the main loop can keep reading K902 (no NODATA).
// After a few AT misses, back off so SET is not held low every 4 s (that kills RX).
void hc12Task(void *) {
  uint8_t atFails = 0;
  for (;;) {
    if (!hc12.atOk() && hc12Lock(1000)) {
      const bool ok = hc12.configure();
      hc12Unlock();
      if (ok) atFails = 0;
      else if (atFails < 20) atFails++;
      Serial.println(ok ? "HC-12 115200 ready" : "HC-12 not 115200 yet — retry");
    }
    const uint32_t waitMs = hc12.atOk() ? 30000UL : (atFails < 3 ? 4000UL : 60000UL);
    vTaskDelay(pdMS_TO_TICKS(waitMs));
  }
}

void onRtcmFromBoard(const uint8_t *data, size_t len) {
  rtcmOutFrames++;
  if ((!baseActive && !rawLogActive) || len == 0) return;
  lastRtcmOutMs = millis();
  if (isBaseRole() && baseActive && hc12.ready() && hc12Lock(0)) {
    hc12.write(data, len);
    hc12Unlock();
  }
  if (deviceConnected && (rawLogActive || !isBaseRole())) queueRtcmToPhone(data, len);
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

// Boot setup can be lost when the K902 restarts mid-queue. NMEA from the saved
// config then keeps flowing, so the normal retry never runs and the bubble stays dead.
void ensureBubbleLog(unsigned long now) {
  if (!TILT_ENABLED || !gnss.nmeaFlowing()) return;
  if (lastImuMs != 0 && now - lastImuMs < 2000) return;
  if (lastImuRearmMs != 0 && now - lastImuRearmMs < 8000) return;
  if (gnss.freeCommandSlots() < 3) return;
  lastImuRearmMs = now;
  gnss.sendCommand("unlog " K902_PORT " rawimub");
  gnss.sendCommand("log " K902_PORT " imudatab ontime 0.05");
#if K902_INS_ENABLED
  gnss.sendCommand("log " K902_PORT " insrtkcompensateb ontime 0.2");
#endif
  Serial.println("IMU quiet - re-armed IMU/INS logs");
}

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
  "log " K902_PORT " imudatab ontime 0.05",  // keep bubble alive while base is transmitting
};

// Queue the K902 RTCM log set + surveyed antenna phase-center, then mark base TX active.
// The fix is remembered in NVS so the BASE-only build can auto-start on it after a reboot.
bool startBaseArp(double lat, double lon, double arp) {
  const size_t needed = sizeof(kBaseCommands) / sizeof(kBaseCommands[0]) + 1;
  if (gnss.freeCommandSlots() < needed) {
    Serial.println("BCMD K902 command queue busy");
    return false;
  }
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
  if (headRole != HeadRole::Base) {
    headRole = HeadRole::Base;
    prefs.putUChar("role", static_cast<uint8_t>(HeadRole::Base));
    Serial.println("Role promoted to BASE");
  }
  Serial.printf("Base ON fix %.9f %.9f %.3f\n", lat, lon, arp);
  return true;
}

bool startBaseAtPosition(double lat, double lon, double alt, double antH) {
  return startBaseArp(lat, lon, alt + antH);
}

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
  if (!startBaseAtPosition(lat, lon, elev, antH)) return false;
  // Only an explicit phone $BCMD,START persists the point — auto-start on the
  // live fix must not pin the base to wherever the head happened to sit.
  prefs.putDouble("bLat", lat);
  prefs.putDouble("bLon", lon);
  prefs.putDouble("bArp", elev + antH);
  Serial.printf("zone %d%c\n", zone, northern ? 'N' : 'S');
  return true;
}

#if FORCE_BASE_ROLE
// BASE-only build: BLE stays up for a long config window so the app can
// survey/average a point (up to ~30 min) and send $BCMD,START. Past the
// window, once the K902 has a position fix and no phone is attached, the
// head starts base TX on its own — reusing an app-surveyed point when one
// was saved, otherwise its live GNSS position. An explicit stop holds
// auto-start off until the next reboot.
const unsigned long kBaseConfigWindowMs = 40UL * 60UL * 1000UL;
unsigned long baseAutoAt = 0;
bool baseAutoHold = false;

void maybeAutoStartBase(unsigned long now) {
  // A connected phone means the app is still configuring — never cut in.
  if (baseActive || baseAutoHold || baseAutoAt == 0 || now < baseAutoAt || deviceConnected) return;
  double lat, lon, arp;
  if (prefs.isKey("bLat") && prefs.isKey("bLon") && prefs.isKey("bArp")) {
    // A surveyed point was set before — reuse it even if the head moved.
    // No fix needed: the K902 gets "fix position" and tracks satellites itself.
    lat = prefs.getDouble("bLat", 0);
    lon = prefs.getDouble("bLon", 0);
    arp = prefs.getDouble("bArp", 0);
    Serial.println("Auto BASE start - saved fix");
  } else {
    const K902Fix &fix = gnss.fix();
    if (!fix.hasPosition()) return;  // keep waiting for a fix past the deadline
    const double antH = prefs.getFloat("baseAntH", poleHeightM);
    lat = fix.latitude;
    lon = fix.longitude;
    arp = fix.altitude + antH;
    Serial.println("Auto BASE start - using own GNSS fix");
  }
  baseAutoAt = 0;
  if (startBaseArp(lat, lon, arp)) {
    bleCutPending = true;
    bleCutAt = millis();
  } else {
    baseAutoAt = now + 10000;  // K902 queue busy - retry shortly
  }
}
#endif

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
    if (strncmp(kK902Setup[i], "unlogall", 8) == 0 || strcmp(kK902Setup[i], "fix none") == 0 ||
        strcmp(kK902Setup[i], "saveconfig") == 0) continue;
    if (!gnss.sendCommand(kK902Setup[i])) Serial.printf("K902 queue full at %s\n", kK902Setup[i]);
  }
  baseActive = false;
#if FORCE_BASE_ROLE
  baseAutoHold = true;  // explicit stop wins over auto-start until reboot
#endif
  // If the queue is too full now, the re-arm check in loop() retries in 20 s.
  if (rawLogActive) armRawLog();
  bleStart();  // no-op while the radio is already up; restores BLE after a base TX cut
  Serial.println("Base OFF");
}

void flushHc12Rx() {
  if (!hc12Lock(20)) return;
  uint8_t dump[64];
  while (hc12.read(dump, sizeof(dump)) > 0) {}
  hc12Unlock();
}

void sendRoleAck() {
  char line[24];
  snprintf(line, sizeof(line), "$ROLE,ACK,%s\n", headRoleName());
  bleSend(line, strlen(line));
}

void applyHeadRole(HeadRole next, bool persist) {
#if FORCE_BASE_ROLE
  if (next == HeadRole::Rover) {
    Serial.println("Base-only build: role stays BASE");
    return;
  }
#endif
  if (next == HeadRole::Rover && baseActive) stopBase();
  if (next != headRole) {
    flushHc12Rx();
    lastRadioRtcmMs = 0;
    lastRadioStatusMs = 0;
    lastBaseLinkStatusMs = 0;
    Serial.printf("Role %s -> %s\n", headRoleName(), next == HeadRole::Base ? "BASE" : "ROVER");
  }
  headRole = next;
  if (persist) prefs.putUChar("role", static_cast<uint8_t>(headRole));
}

void sendBstatOff() {
  const char *line = "$BSTAT,OFF,0,0\n";
  bleSend(line, strlen(line));
}

void applyInsRodLength(float meters) {
  if (meters < 0.1f) meters = 0.1f;
  if (meters > 20.0f) meters = 20.0f;
  poleHeightM = meters;
  prefs.putFloat("poleH", poleHeightM);
}

#if K902_INS_ENABLED
void onInsRtkComp(const K902InsRtk &c) {
  insRtkLive = true;
  // Bubble stays on high-rate IMU fuse — INS attitude (~5 Hz) feels laggy in the hand.
  if (isfinite(c.tilt)) tiltDeg = c.tilt;

  if (insInitPending && c.status != lastInsStatus) {
    lastInsStatus = c.status;
    char line[48];
    if (c.status == 4) {
      insInitPending = false;
      setLevelReference();
      snprintf(line, sizeof(line), "$ICALIB,OK,ins\n");
    } else if (c.status == 2 || c.status == 3) {
      snprintf(line, sizeof(line), "$ICALIB,ALIGN,%d,%u\n", (int)c.status, (unsigned)c.step);
    } else if (c.status == -1) {
      snprintf(line, sizeof(line), "$ICALIB,START,wait_init\n");
    } else {
      snprintf(line, sizeof(line), "$ICALIB,ALIGN,%d,%u\n", (int)c.status, (unsigned)c.step);
    }
    bleSend(line, strlen(line));
  }

  unsigned long now = millis();
  if (deviceConnected && now - lastPinsMs >= 200) {
    lastPinsMs = now;
    char line[128];
    snprintf(line, sizeof(line), "$PINS,%d,%.2f,%.2f,%.2f,%u,%u,0x%X\n",
             (int)c.status, c.tilt, c.pitch, c.roll,
             c.compensateValid ? 1u : 0u, (unsigned)c.step, (unsigned)c.errorCode);
    bleSend(line, strlen(line));
    if (c.compensateValid) {
      // App-side guide/stakeout also needs heading together with the live
      // bubble.  Convert the bubble's IMU frame to the app's pole frame.
      char tiltLine[80];
      const float polePitch = -shownRoll;
      const float poleRoll = shownPitch;
      snprintf(tiltLine, sizeof(tiltLine), "$PTILT,%.2f,%.2f,%.2f\n", polePitch, poleRoll, c.heading);
      bleSend(tiltLine, strlen(tiltLine));
      // Tip ≈ ARP + ENU deltas (small-angle flat Earth).
      const double mPerDegLat = 111132.92;
      const double mPerDegLon = 111412.84 * cos(c.lat * DEG_TO_RAD);
      const double tipLat = c.lat + c.deltaNorth / mPerDegLat;
      const double tipLon = c.lon + (mPerDegLon != 0.0 ? c.deltaEast / mPerDegLon : 0.0);
      const double tipAlt = c.hgt + c.deltaUp;
      snprintf(line, sizeof(line), "$PTIP,%.9f,%.9f,%.3f,%.2f\n", tipLat, tipLon, tipAlt, c.tilt);
      bleSend(line, strlen(line));
    }
  }
}
#endif

void sendFwVersion() {
  char line[32];
  const int n = snprintf(line, sizeof(line), "$FW,%s\n", FW_LABEL);
  if (n > 0) bleSend(line, (size_t)n);
}

void handlePhoneCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) {
    if (*p == '\r' || *p == '\n') *p = '\0';
  }
  Serial.printf("Phone< %s\n", cmd);
  if (strcmp(cmd, "$FW") == 0 || strncmp(cmd, "$FW,", 4) == 0 ||
      strcmp(cmd, "$VER") == 0 || strncmp(cmd, "$VER,", 5) == 0) {
    sendFwVersion();
    return;
  }
  if (strncmp(cmd, "$WCMD,WIFI,", 11) == 0) {
    bleSend("$WCMD,ACK\n", 10);
    return;
  }
  if (strncmp(cmd, "$ROLE", 5) == 0) {
    const char *arg = (cmd[5] == ',') ? cmd + 6 : "";
    if (strcmp(arg, "BASE") == 0) applyHeadRole(HeadRole::Base, true);
    else if (strcmp(arg, "ROVER") == 0) applyHeadRole(HeadRole::Rover, true);
    sendRoleAck();
    return;
  }
  if (strncmp(cmd, "$RCMD,SOURCE,", 13) == 0) {
    applyHeadRole(HeadRole::Rover, true);
    const char *source = cmd + 13;
    if (strcmp(source, "BASE") == 0) {
      roverCorrectionSource = RoverCorrectionSource::Radio;
      prefs.putUChar("corrSrc", static_cast<uint8_t>(roverCorrectionSource));
      flushHc12Rx();
      lastRadioRtcmMs = 0;
      bleSend("$RSOURCE,ACK,BASE\n", 18);
    } else if (strcmp(source, "NTRIP") == 0) {
      roverCorrectionSource = RoverCorrectionSource::PhoneNtrip;
      prefs.putUChar("corrSrc", static_cast<uint8_t>(roverCorrectionSource));
      bleSend("$RSOURCE,ACK,NTRIP\n", 19);
    }
    lastBaseLinkStatusMs = 0;
    return;
  }
  if (strncmp(cmd, "$RCMD,BASEHOST,", 15) == 0) {
    bleSend("$BASEHOST,ACK\n", 14);
    return;
  }
  if (strncmp(cmd, "$RCMD,STOPBASE", 14) == 0) {
    if (isBaseRole()) {
      if (baseActive) stopBase();
      sendBstat();
      bleSend("$BSTOP,OK\n", 10);
      Serial.println("STOPBASE on this head — local base off");
      return;
    }
    bool ok = false;
    if (hc12.ready() && hc12Lock(50)) {
      ok = hc12.writeText("$BCMD,STOP\n") > 0;
      hc12Unlock();
    }
    bleSend(ok ? "$BSTOP,OK\n" : "$BSTOP,FAIL\n", ok ? 10 : 12);
    Serial.printf("Rover asked base to stop via HC-12: %s\n", ok ? "OK" : "FAIL");
    return;
  }
  if (strncmp(cmd, "$BCMD,STATUS", 12) == 0) {
    sendBstat();
    return;
  }
  if (strncmp(cmd, "$ICMD,CALIB", 11) == 0) {
    // Optional pole height: $ICMD,CALIB,2.000
    const char *comma = strchr(cmd + 11, ',');
    if (comma && comma[1]) {
      float h = (float)atof(comma + 1);
      if (h > 0.05f) applyInsRodLength(h);
    }
    bool levelOk = setLevelReference();
    applyInsRodLength(poleHeightM);
#if K902_INS_ENABLED
    // The INS log is already armed at boot.  Wait for its first status frame
    // so the app can distinguish a real tilted-pole solution from bubble-only.
    insRtkLive = false;
    insInitPending = levelOk;
    insInitStartedMs = millis();
    lastInsStatus = -99;
    const char *reply = levelOk ? "$ICALIB,START,wait_ins\n" : "$ICALIB,FAIL,no_imu\n";
#else
    const char *reply = levelOk ? "$ICALIB,OK,level_only\n" : "$ICALIB,FAIL,no_imu\n";
#endif
    bleSend(reply, strlen(reply));
    Serial.printf("Phone> %s", reply);
    return;
  }
  if (strncmp(cmd, "$ICMD,HEIGHT", 12) == 0) {
    const char *comma = strchr(cmd + 12, ',');
    if (comma && comma[1]) {
      float h = (float)atof(comma + 1);
      if (h > 0.05f) applyInsRodLength(h);
    }
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
    prefs.putFloat("baseAntH", (float)antH);  // reused by BASE-build auto start
    sendBstat();
    // The base keeps streaming RTCM over HC-12 without the phone. Give the
    // $BSTAT ack a moment to flush, then cut the BLE radio to save power.
    bleCutPending = true;
    bleCutAt = millis();
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
  Serial.printf("[K902] %s %s sats=%u hdop=%.1f", headRoleName(), k902QualityName(fix.quality), fix.satellites, fix.hdop);
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
  Serial.printf(" | HC-12 %lu baud tx=%lu rx=%lu drop=%lu", (unsigned long)hc12.baud(),
                (unsigned long)hc12.txBytes(), (unsigned long)hc12.rxBytes(), (unsigned long)hc12.drops());
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
    } else if (strncmp(line, "!baseh ", 7) == 0) {
      float h = (float)atof(line + 7);
      if (h >= 0.0f && h < 20.0f) {
        prefs.putFloat("baseAntH", h);
        Serial.printf("Base antenna height %.3f m\n", h);
      } else {
        Serial.println("!baseh needs 0 <= h < 20 m");
      }
    } else if (strncmp(line, "!basepos ", 9) == 0) {
      // !basepos <lat> <lon> <arp_m> — survey the base point by hand, no phone needed.
      double blat = 0, blon = 0, barp = 0;
      if (sscanf(line + 9, "%lf %lf %lf", &blat, &blon, &barp) == 3 &&
          blat >= -90 && blat <= 90 && blon >= -180 && blon <= 180 && barp > -500 && barp < 10000) {
        prefs.putDouble("bLat", blat);
        prefs.putDouble("bLon", blon);
        prefs.putDouble("bArp", barp);
        Serial.printf("Saved base fix %.9f %.9f %.3f - auto-start uses it without a GNSS fix\n", blat, blon, barp);
      } else {
        Serial.println("!basepos <lat> <lon> <arp_m>");
      }
    } else if (strcmp(line, "!baseclr") == 0) {
      prefs.remove("bLat");
      prefs.remove("bLon");
      prefs.remove("bArp");
      Serial.println("Saved base fix cleared - auto-start will use the live GNSS position");
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
    fwAnnouncePending = true;
    lastFwAnnounceMs = 0;
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
  // Phone-side supervision timeout does not always deliver onDisconnect, so the
  // head stays "connected" and never advertises. Drop a silent link and advertise.
  if (deviceConnected && lastBleTrafficMs != 0 && now - lastBleTrafficMs > 20000) {
    Serial.println("BLE silent 20s — drop link and advertise");
    const uint16_t id = peerConnId;
    deviceConnected = false;
    peerConnId = 0;
    oldDeviceConnected = false;
    fwAnnouncePending = false;
    advertisePending = true;
    disconnectedAt = now;
    phoneLostAt = now;
    if (id != 0) pServer->disconnect(id);
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
    fwAnnouncePending = false;
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

void otaNotify(const char *msg) {
  if (pOtaCharacteristic == NULL || !deviceConnected || msg == NULL) return;
  pOtaCharacteristic->setValue((uint8_t *)msg, strlen(msg));
  pOtaCharacteristic->notify();
}

void abortOta() {
  if (!otaActive && !Update.isRunning()) return;
  Update.abort();
  otaActive = false;
  otaExpected = 0;
  otaWritten = 0;
  otaRebootPending = false;
}

uint32_t otaSlotCap() {
  const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
  return next == nullptr ? 0 : next->size;
}

bool otaBegin(uint32_t size) {
  const uint32_t cap = otaSlotCap();
  if (cap == 0 || size == 0 || size > cap) {
    otaNotify("$OTA,ERR,size\n");
    return false;
  }
  if (otaActive) abortOta();
  if (!Update.begin(size)) {
    otaNotify("$OTA,ERR,begin\n");
    return false;
  }
  otaActive = true;
  otaExpected = size;
  otaWritten = 0;
  otaRebootPending = false;
  otaNotify("$OTA,ACK,0\n");
  return true;
}

void otaWriteChunk(const uint8_t *data, size_t len) {
  if (!otaActive || data == nullptr || len == 0) return;
  if (otaWritten + len > otaExpected) {
    abortOta();
    otaNotify("$OTA,ERR,over\n");
    return;
  }
  const size_t n = Update.write(const_cast<uint8_t *>(data), len);
  if (n != len) {
    abortOta();
    otaNotify("$OTA,ERR,write\n");
    return;
  }
  otaWritten += n;
}

void otaFinish() {
  if (!otaActive) return;
  if (otaWritten != otaExpected) {
    abortOta();
    otaNotify("$OTA,ERR,short\n");
    return;
  }
  if (!Update.end(true)) {
    abortOta();
    otaNotify("$OTA,ERR,end\n");
    return;
  }
  otaActive = false;
  otaNotify("$OTA,OK\n");
  otaRebootAt = millis();
  otaRebootPending = true;
}

class OtaCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *data = c->getData();
    const size_t len = c->getLength();
    if (data == nullptr || len == 0) return;
    const uint8_t op = data[0];
    if (op == kOtaStart && len >= 5) {
      const uint32_t size = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                            ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
      otaBegin(size);
      return;
    }
    if (!otaActive) return;
    if (op == kOtaAbort) {
      abortOta();
      otaNotify("$OTA,ERR,abort\n");
      return;
    }
    if (op == kOtaFinish) {
      otaFinish();
      return;
    }
    if (op == kOtaChunk && len > 1) otaWriteChunk(data + 1, len - 1);
  }
};

// BLE + Classic SPP bring-up. Safe to call again after bleStop() — it rebuilds
// the whole GATT server and restarts advertising.
void bleStart() {
  if (pServer != NULL) return;
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
  pOtaCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_OTA,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pOtaCharacteristic->addDescriptor(new BLE2902());
  pOtaCharacteristic->setCallbacks(new OtaCallbacks());
  pService->start();
  rxCharHandle = pRxCharacteristic->getHandle();
  Serial.printf("RX char %s handle=%u (WRITE + WRITE_NR)\n",
                pRxCharacteristic->getUUID().toString().c_str(), (unsigned)rxCharHandle);
  BLEAdvertising *adv = pServer->getAdvertising();
  // The 128-bit NUS UUID does not fit beside "E25aer_gnss" in the 31-byte
  // advert. Putting the UUID here pushes the name into the scan response,
  // which the phone often misses, so the survey list stays empty.
  adv->setScanResponse(true);
  BLEAdvertisementData scanData;
  scanData.setCompleteServices(BLEUUID(SERVICE_UUID));
  char mfg[2 + sizeof(FW_LABEL)];
  mfg[0] = (char)0xFF;
  mfg[1] = (char)0xFF;
  memcpy(mfg + 2, FW_LABEL, strlen(FW_LABEL));
  scanData.setManufacturerData(String(mfg, 2 + strlen(FW_LABEL)));
  adv->setScanResponseData(scanData);
  adv->start();
  Serial.println("Waiting for phone...");
}

// Kill the whole Bluetooth radio (BLE + Classic SPP) so a transmitting base
// head does not carry the ~50 mA phone link. stopBase() brings it back.
void bleStop() {
  if (pServer == NULL) {
    SerialBT.end();
    return;
  }
  const uint16_t connId = peerConnId;
  deviceConnected = false;
  oldDeviceConnected = false;
  peerConnId = 0;
  advertisePending = false;
  pServer->getAdvertising()->stop();
  if (connId != 0) pServer->disconnect(connId);
  delay(100);  // let the disconnect reach the phone before the stack goes down
  BLEDevice::deinit(true);
  SerialBT.end();
  pServer = NULL;
  pTxCharacteristic = NULL;
  pTiltCharacteristic = NULL;
  pOtaCharacteristic = NULL;
  Serial.println("BLE radio off - base TX continues on HC-12");
}

void setup() {
  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, LOW);  // hold AT before USB/Serial come up
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.begin(115200);
  Serial.printf("\nReset reason: %s\n", resetReasonName(esp_reset_reason()));
  lastResetReason = (int)esp_reset_reason();
  rtcmMux = xSemaphoreCreateMutex();
  phoneCmdQueue = xQueueCreate(4, kPhoneCmdMax);
  rtcmBuffer = xStreamBufferCreate(4096, 1);
  prefs.begin("rover");
#if FORCE_BASE_ROLE
  headRole = HeadRole::Base;
  baseAutoAt = millis() + kBaseConfigWindowMs;  // app may still send $BCMD,START first
#else
  headRole = prefs.getUChar("role", 0) == 1 ? HeadRole::Base : HeadRole::Rover;
#endif
  const uint8_t savedSource = prefs.getUChar("corrSrc", 0);
  roverCorrectionSource = savedSource == 1 ? RoverCorrectionSource::PhoneNtrip : RoverCorrectionSource::Radio;
  Serial.printf("E25aer_gnss fw %s role %s\n", FW_LABEL, headRoleName());
  if (prefs.isKey("poleH")) {
    float h = prefs.getFloat("poleH", 2.0f);
    if (h > 0.05f && h < 20.0f) poleHeightM = h;
  }
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
#if K902_INS_ENABLED
  gnss.onInsRtk(onInsRtkComp);
#endif
  gnss.onRtcmOut(onRtcmFromBoard);
  gnss.begin(Serial2, K902_RX_PIN, K902_TX_PIN, K902_BAUD, kK902Setup,
             sizeof(kK902Setup) / sizeof(kK902Setup[0]), K902_RTCM_INPUT);

  hc12Mux = xSemaphoreCreateMutex();
  hc12.begin(HC12_RX_PIN, HC12_TX_PIN, HC12_SET_PIN);
  xTaskCreate(hc12Task, "hc12", 4096, nullptr, 1, nullptr);

  bleStart();
}

void loop() {
  unsigned long now = millis();
  if (bleCutPending && now - bleCutAt >= 600) {
    bleCutPending = false;
    bleStop();
  }
  if (otaRebootPending && now - otaRebootAt >= 400) {
    otaRebootPending = false;
    ESP.restart();
  }
  serviceBleCongestion(now);
  serviceBle(now);
  if (fwAnnouncePending && deviceConnected && now - lastFwAnnounceMs >= 500) {
    lastFwAnnounceMs = now;
    char line[32];
    const int n = snprintf(line, sizeof(line), "$FW,%s\n", FW_LABEL);
    if (n > 0 && bleSend(line, (size_t)n)) fwAnnouncePending = false;
  }
  bool radioStop = false;
  if (hc12Lock(0)) {
    if (isBaseRole() && hc12.ready()) {
      char radioLine[80];
      if (hc12.pollLine(radioLine, sizeof(radioLine)) && strncmp(radioLine, "$BCMD,STOP", 10) == 0) {
        radioStop = true;
      }
    } else if (!isBaseRole() && hc12.ready() &&
               roverCorrectionSource == RoverCorrectionSource::Radio && !baseActive) {
      uint8_t baseRtcm[256];
      size_t baseRtcmLen;
      while ((baseRtcmLen = hc12.read(baseRtcm, sizeof(baseRtcm))) > 0) {
        lastRadioRtcmMs = now;
        rtcmPush(baseRtcm, baseRtcmLen);
      }
    }
    hc12Unlock();
  }
  if (radioStop && baseActive) {
    Serial.println("HC-12 stop from rover");
    stopBase();
  }
  if (sppUpEvent) {
    sppUpEvent = false;
    Serial.println("Survey Master Bluetooth connected");
  }
  if (sppDownEvent) {
    sppDownEvent = false;
    Serial.println("Survey Master Bluetooth disconnected");
  }
  drainSpp();
  if (isBaseRole()) {
    if (deviceConnected && now - lastRadioStatusMs >= 2000) {
      lastRadioStatusMs = now;
      char line[48];
      if (baseActive && hc12.ready()) snprintf(line, sizeof(line), "$RADIO,UP,%lu\n", (unsigned long)hc12.txBytes());
      else if (hc12.atOk()) snprintf(line, sizeof(line), "$RADIO,READY,115200\n");
      else if (hc12.ready()) snprintf(line, sizeof(line), "$RADIO,READY,NOAT\n");
      else snprintf(line, sizeof(line), "$RADIO,WAIT,AT\n");
      bleSend(line, strlen(line));
    }
  } else {
  if (deviceConnected && now - lastBaseLinkStatusMs >= 2000) {
    lastBaseLinkStatusMs = now;
    const char *linkStatus;
    if (roverCorrectionSource == RoverCorrectionSource::PhoneNtrip) {
      linkStatus = "$BASELINK,MODE,NTRIP\n";
    } else if ((now - lastRadioRtcmMs) < 3000 && lastRadioRtcmMs != 0) {
      linkStatus = "$BASELINK,UP\n";
    } else if (!hc12.atOk()) {
      linkStatus = "$BASELINK,WAIT,AT\n";
    } else {
      linkStatus = "$BASELINK,WAIT,RADIO\n";
    }
    bleSend(linkStatus, strlen(linkStatus));
  }
  }
  handleConsole();
  char cmd[kPhoneCmdMax];
  while (phoneCmdQueue && xQueueReceive(phoneCmdQueue, cmd, 0) == pdTRUE) {
    cmd[kPhoneCmdMax - 1] = '\0';
    handlePhoneCommand(cmd);
  }
#if K902_INS_ENABLED
  // Boards without INSRTK never emit compensate frames — fall back to bubble level-only OK.
  if (insInitPending && !insRtkLive && insInitStartedMs != 0 && now - insInitStartedMs > 85000) {
    insInitPending = false;
    const char *reply = setLevelReference() ? "$ICALIB,OK,level_only\n" : "$ICALIB,FAIL,no_imu\n";
    bleSend(reply, strlen(reply));
    Serial.println("INS-RTK not reporting — used bubble level reference only");
  }
#endif
  if (!isBaseRole() && baseActive) {
    static unsigned long lastRoleStopMs = 0;
    if (now - lastRoleStopMs >= 2000) {
      lastRoleStopMs = now;
      stopBase();
    }
  }
  // Base role keeps sending RTCM over HC-12 after the phone disconnects BLE.
  if (isBaseRole()) {
    if (!oldDeviceConnected && now - phoneLostAt >= 60000 && rawLogActive && !baseActive) {
      Serial.println("Phone gone 60 s - stopping raw log");
      disarmRawLog();
      rawLogActive = false;
    }
  } else if (!oldDeviceConnected && now - phoneLostAt >= 60000) {
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

  ensureBubbleLog(now);
#if FORCE_BASE_ROLE
  maybeAutoStartBase(now);
#endif
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
    if (BLE_SEND_PRTK_S && (isBaseRole() ? deviceConnected : !baseActive)) sendPrtk();
    if ((isBaseRole() ? deviceConnected : baseActive) && now - lastBstatMs >= 1000) {
      lastBstatMs = now;
      sendBstat();
    }
    if (rawLogActive && now - lastLstatMs >= 5000) sendLstat();
  }
  delay(1);
}
