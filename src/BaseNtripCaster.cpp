#include "BaseNtripCaster.h"

#include <string.h>

namespace {
const char kSourceTableHead[] =
    "SOURCETABLE 200 OK\r\n"
    "Server: K902 Base\r\n"
    "Content-Type: text/plain\r\n"
    "\r\n";
}

void BaseNtripCaster::setCredentials(const char *ssid, const char *pass) {
  if (ssid == nullptr || !ssid[0]) return;
  const bool changed = strcmp(ssid_, ssid) != 0 || strcmp(pass_, pass ? pass : "") != 0;
  strlcpy(ssid_, ssid, sizeof(ssid_));
  strlcpy(pass_, pass ? pass : "", sizeof(pass_));
  if (changed && wanted_) {
    closeClient();
    if (udpBound_) {
      udp_.stop();
      udpBound_ = false;
    }
    if (serverStarted_) {
      server_.stop();
      serverStarted_ = false;
    }
    WiFi.disconnect(true, false);
    nextJoinMs_ = 0;
  }
}

bool BaseNtripCaster::begin() {
  if (!hasCredentials()) {
    Serial.println("Base Wi-Fi: no phone hotspot SSID");
    return false;
  }
  wanted_ = true;
  nextJoinMs_ = 0;
  ensureStation();
  return true;
}

void BaseNtripCaster::end() {
  wanted_ = false;
  closeClient();
  if (udpBound_) {
    udp_.stop();
    udpBound_ = false;
  }
  if (serverStarted_) {
    server_.stop();
    serverStarted_ = false;
  }
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  nextJoinMs_ = 0;
}

void BaseNtripCaster::ensureStation() {
  if (!wanted_ || !hasCredentials()) return;
  const unsigned long now = millis();
  if (WiFi.status() == WL_CONNECTED) return;
  if (now < nextJoinMs_) return;
  const wl_status_t st = WiFi.status();
  Serial.printf("Wi-Fi %s status=%d heap=%lu\n", ssid_, (int)st, (unsigned long)ESP.getFreeHeap());
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("GNSS-BASE");
  WiFi.setMinSecurity(WIFI_AUTH_OPEN);
  WiFi.begin(ssid_, pass_);
  nextJoinMs_ = now + 12000;
  Serial.printf("Base joining phone hotspot %s\n", ssid_);
}

void BaseNtripCaster::service(bool baseActive) {
  if (!wanted_) return;
  ensureStation();
  if (WiFi.status() != WL_CONNECTED) return;

  if (!serverStarted_) {
    server_.begin();
    server_.setNoDelay(true);
    serverStarted_ = true;
    Serial.printf("Base NTRIP on hotspot IP=%s:%u mount=BASE\n",
                  WiFi.localIP().toString().c_str(), kPort);
  }
  if (!udpBound_) {
    udpBound_ = udp_.begin(kBeaconPort);
  }

  sendBeacon(millis());
  acceptClient();
  if (!client_) return;

  if (!streaming_) {
    readRequest(baseActive);
    if (!streaming_ || !client_) return;
  }
  if (!baseActive) {
    closeClient();
    return;
  }

  if (!client_.connected() && client_.available() == 0) {
    closeClient();
    return;
  }
  int rxBudget = 256;
  while (rxBudget-- > 0 && client_.available() > 0) client_.read();
  sendRtcmChunk();
}

void BaseNtripCaster::sendBeacon(unsigned long now) {
  if (!udpBound_ || now - lastBeaconMs_ < 2000) return;
  lastBeaconMs_ = now;
  char msg[64];
  snprintf(msg, sizeof(msg), "K902BASE,%s,2101,BASE", WiFi.localIP().toString().c_str());
  IPAddress bcast = WiFi.localIP();
  bcast[3] = 255;
  udp_.beginPacket(bcast, kBeaconPort);
  udp_.write(reinterpret_cast<const uint8_t *>(msg), strlen(msg));
  udp_.endPacket();
}

void BaseNtripCaster::acceptClient() {
  WiFiClient incoming = server_.accept();
  if (!incoming) return;

  if (client_ && streaming_) {
    incoming.print("HTTP/1.0 503 Caster busy\r\nConnection: close\r\n\r\n");
    incoming.stop();
    return;
  }
  if (client_) closeClient();

  client_ = incoming;
  client_.setNoDelay(true);
  requestLineLen_ = 0;
  requestLine_[0] = '\0';
  requestStartedMs_ = millis();
  lastTxProgressMs_ = requestStartedMs_;
  streaming_ = false;
  clearQueue();
}

void BaseNtripCaster::readRequest(bool baseActive) {
  if (millis() - requestStartedMs_ > 5000) {
    rejectRequest(408, "Request timeout");
    return;
  }

  int budget = 128;
  while (budget-- > 0 && client_.available() > 0) {
    const int value = client_.read();
    if (value < 0) break;
    const char c = static_cast<char>(value);
    if (c == '\n') {
      requestLine_[requestLineLen_] = '\0';
      handleRequestLine(baseActive);
      return;
    }
    if (c == '\r') continue;
    if (requestLineLen_ + 1 >= sizeof(requestLine_)) {
      rejectRequest(400, "Request too long");
      return;
    }
    requestLine_[requestLineLen_++] = c;
  }
}

void BaseNtripCaster::handleRequestLine(bool baseActive) {
  char *path = strchr(requestLine_, ' ');
  if (path == nullptr) {
    rejectRequest(400, "Bad request");
    return;
  }
  *path++ = '\0';
  if (strcmp(requestLine_, "GET") != 0) {
    rejectRequest(405, "GET required");
    return;
  }
  char *end = strchr(path, ' ');
  if (end != nullptr) *end = '\0';

  if (strcmp(path, "/") == 0) {
    sendSourceTable();
    return;
  }
  if (strcmp(path, "/BASE") != 0) {
    rejectRequest(404, "Mountpoint not found");
    return;
  }
  if (!baseActive) {
    rejectRequest(503, "Base inactive");
    return;
  }

  client_.print("ICY 200 OK\r\nNtrip-Version: Ntrip/2.0\r\nServer: K902 Base\r\n\r\n");
  streaming_ = true;
  lastTxProgressMs_ = millis();
  clearQueue();
  Serial.printf("NTRIP client connected from %s\n", client_.remoteIP().toString().c_str());
}

void BaseNtripCaster::sendSourceTable() {
  client_.print(kSourceTableHead);
  client_.printf(
      "STR;BASE;K902 Base;RTCM 3;1005(5),1033(10),1074(1),1084(1),1094(1),1124(1);2;"
      "GPS+GLO+GAL+BDS;NONE;N;N;0;0;ESP32;%s:2101\r\nENDSOURCETABLE\r\n",
      WiFi.localIP().toString().c_str());
  client_.stop();
  streaming_ = false;
  clearQueue();
}

void BaseNtripCaster::rejectRequest(uint16_t status, const char *reason) {
  client_.printf("HTTP/1.0 %u %s\r\nConnection: close\r\nContent-Length: 0\r\n\r\n",
                 status, reason);
  client_.stop();
  streaming_ = false;
  clearQueue();
}

void BaseNtripCaster::pushRtcm(const uint8_t *data, size_t len) {
  if (!streaming_ || !client_ || data == nullptr || len == 0 || len > kFrameMax) return;
  if (queueCount_ >= kQueueDepth) return;

  const uint8_t tail = static_cast<uint8_t>((queueHead_ + queueCount_) % kQueueDepth);
  RtcmFrame &frame = queue_[tail];
  memcpy(frame.bytes, data, len);
  frame.len = static_cast<uint16_t>(len);
  frame.sent = 0;
  queueCount_++;
}

void BaseNtripCaster::sendRtcmChunk() {
  if (queueCount_ == 0) return;
  RtcmFrame &frame = queue_[queueHead_];
  const size_t remaining = frame.len - frame.sent;
  const size_t chunk = remaining < kSendChunk ? remaining : kSendChunk;
  const size_t written = client_.write(frame.bytes + frame.sent, chunk);
  if (written > 0) {
    frame.sent += static_cast<uint16_t>(written);
    lastTxProgressMs_ = millis();
    if (frame.sent >= frame.len) {
      frame.len = 0;
      frame.sent = 0;
      queueHead_ = static_cast<uint8_t>((queueHead_ + 1) % kQueueDepth);
      queueCount_--;
    }
    return;
  }
  if (millis() - lastTxProgressMs_ > 10000) {
    Serial.println("NTRIP client stalled; closing connection");
    closeClient();
  }
}

void BaseNtripCaster::closeClient() {
  client_.stop();
  streaming_ = false;
  requestLineLen_ = 0;
  requestLine_[0] = '\0';
  clearQueue();
}

void BaseNtripCaster::clearQueue() {
  queueHead_ = 0;
  queueCount_ = 0;
}
