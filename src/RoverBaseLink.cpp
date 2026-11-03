#include "RoverBaseLink.h"

#include <string.h>

namespace {
constexpr uint16_t kBaseNtripPort = 2101;
constexpr uint16_t kBeaconPort = 2102;
constexpr unsigned long kWifiRetryMs = 12000;
constexpr unsigned long kCasterRetryMs = 4000;
constexpr unsigned long kHeaderTimeoutMs = 5000;
}

void RoverBaseLink::setCredentials(const char *ssid, const char *pass) {
  if (ssid == nullptr || !ssid[0]) return;
  strlcpy(ssid_, ssid, sizeof(ssid_));
  strlcpy(pass_, pass ? pass : "", sizeof(pass_));
}

void RoverBaseLink::setBaseHost(const char *ip) {
  if (ip == nullptr || !ip[0]) return;
  strlcpy(host_, ip, sizeof(host_));
}

void RoverBaseLink::begin() {
  if (enabled_) return;
  enabled_ = true;
  lastWifiAttemptMs_ = 0;
  state_ = State::WaitingWifi;
}

void RoverBaseLink::end() {
  if (!enabled_) return;
  enabled_ = false;
  client_.stop();
  if (udpBound_) {
    udp_.stop();
    udpBound_ = false;
  }
  responseHeaderLen_ = 0;
  responseHeaders_[0] = '\0';
  state_ = State::Disabled;
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
}

void RoverBaseLink::ensureStation(unsigned long now) {
  if (!hasCredentials()) return;
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWifiAttemptMs_ != 0 && now - lastWifiAttemptMs_ < kWifiRetryMs) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid_, pass_);
  lastWifiAttemptMs_ = now;
  Serial.printf("Rover joining phone hotspot %s\n", ssid_);
}

void RoverBaseLink::pollBeacon() {
  if (!udpBound_) return;
  int size = udp_.parsePacket();
  if (size <= 0) return;
  char buf[64];
  const int n = udp_.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  if (strncmp(buf, "K902BASE,", 9) != 0) return;
  char ip[16] = {};
  if (sscanf(buf + 9, "%15[0-9.]", ip) == 1 && ip[0]) {
    if (strcmp(host_, ip) != 0) {
      strlcpy(host_, ip, sizeof(host_));
      Serial.printf("Rover heard base IP %s\n", host_);
    }
  }
}

void RoverBaseLink::service(unsigned long now) {
  if (!enabled_) return;
  ensureStation(now);
  if (WiFi.status() != WL_CONNECTED) {
    client_.stop();
    state_ = State::WaitingWifi;
    responseHeaderLen_ = 0;
    responseHeaders_[0] = '\0';
    return;
  }

  if (!udpBound_) udpBound_ = udp_.begin(kBeaconPort);
  pollBeacon();

  if (state_ == State::WaitingWifi) {
    state_ = State::WaitingCaster;
    nextCasterAttemptMs_ = now;
  }

  if (state_ == State::Streaming) {
    if (!client_.connected() && client_.available() == 0) {
      client_.stop();
      state_ = State::WaitingCaster;
      nextCasterAttemptMs_ = now + kCasterRetryMs;
    }
    return;
  }

  if (state_ == State::WaitingCaster) {
    if (static_cast<int32_t>(now - nextCasterAttemptMs_) < 0) return;
    startCasterConnection(now);
    return;
  }

  if (state_ != State::ReadingHeaders) return;
  if (now - responseStartedMs_ > kHeaderTimeoutMs) {
    rejectCasterConnection(now);
    return;
  }

  int budget = 256;
  while (budget-- > 0 && client_.available() > 0) {
    const int value = client_.read();
    if (value < 0) break;
    if (responseHeaderLen_ + 1 >= sizeof(responseHeaders_)) {
      rejectCasterConnection(now);
      return;
    }
    responseHeaders_[responseHeaderLen_++] = static_cast<char>(value);
    responseHeaders_[responseHeaderLen_] = '\0';
    if (responseHeaderLen_ < 4 ||
        memcmp(responseHeaders_ + responseHeaderLen_ - 4, "\r\n\r\n", 4) != 0) {
      continue;
    }

    const bool accepted =
        strncmp(responseHeaders_, "ICY 200 OK", 10) == 0 ||
        strncmp(responseHeaders_, "HTTP/1.0 200", 12) == 0 ||
        strncmp(responseHeaders_, "HTTP/1.1 200", 12) == 0;
    if (accepted) {
      state_ = State::Streaming;
      responseHeaderLen_ = 0;
      responseHeaders_[0] = '\0';
    } else {
      rejectCasterConnection(now);
    }
    return;
  }

  if (!client_.connected() && client_.available() == 0) {
    rejectCasterConnection(now);
  }
}

size_t RoverBaseLink::read(uint8_t *buffer, size_t capacity) {
  if (!streaming() || buffer == nullptr || capacity == 0) return 0;
  const int received = client_.read(buffer, capacity);
  return received > 0 ? static_cast<size_t>(received) : 0;
}

bool RoverBaseLink::streaming() const {
  return state_ == State::Streaming;
}

bool RoverBaseLink::stopRemoteBase() {
  if (WiFi.status() != WL_CONNECTED || !host_[0]) return false;
  WiFiClient http;
  IPAddress addr;
  if (!addr.fromString(host_)) return false;
  if (!http.connect(addr, 80, 2000)) return false;
  http.printf("POST /api/base/stop HTTP/1.0\r\nHost: %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
              host_);
  char buf[160];
  size_t n = 0;
  const unsigned long start = millis();
  while (millis() - start < 3000) {
    while (http.available() > 0 && n + 1 < sizeof(buf)) {
      buf[n++] = static_cast<char>(http.read());
    }
    buf[n] = '\0';
    if (strstr(buf, "HTTP/1.") != nullptr && strstr(buf, " 200") != nullptr) {
      http.stop();
      return true;
    }
    if (!http.connected() && http.available() == 0) break;
    delay(10);
  }
  http.stop();
  return n > 0 && strstr(buf, " 200") != nullptr;
}

void RoverBaseLink::startCasterConnection(unsigned long now) {
  client_.stop();
  if (!host_[0]) {
    nextCasterAttemptMs_ = now + kCasterRetryMs;
    return;
  }
  IPAddress addr;
  if (!addr.fromString(host_) || !client_.connect(addr, kBaseNtripPort, 1500)) {
    nextCasterAttemptMs_ = now + kCasterRetryMs;
    return;
  }

  client_.setNoDelay(true);
  client_.print("GET /BASE HTTP/1.0\r\n"
                "User-Agent: NTRIP ESP32-Rover\r\n"
                "Connection: keep-alive\r\n\r\n");
  responseHeaderLen_ = 0;
  responseHeaders_[0] = '\0';
  responseStartedMs_ = now;
  nextCasterAttemptMs_ = now + kCasterRetryMs;
  state_ = State::ReadingHeaders;
}

void RoverBaseLink::rejectCasterConnection(unsigned long now) {
  client_.stop();
  responseHeaderLen_ = 0;
  responseHeaders_[0] = '\0';
  state_ = State::WaitingCaster;
  nextCasterAttemptMs_ = now + kCasterRetryMs;
}
