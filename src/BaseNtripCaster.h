#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Base joins the phone hotspot as STA and serves RTCM on :2101 /BASE.
class BaseNtripCaster {
 public:
  void setCredentials(const char *ssid, const char *pass);
  bool hasCredentials() const { return ssid_[0] != '\0'; }
  bool begin();
  void end();
  void service(bool baseActive);
  void pushRtcm(const uint8_t *data, size_t len);
  bool wifiUp() const { return WiFi.status() == WL_CONNECTED; }
  IPAddress localIp() const { return WiFi.localIP(); }

 private:
  static constexpr uint16_t kPort = 2101;
  static constexpr uint16_t kBeaconPort = 2102;
  static constexpr size_t kFrameMax = 1029;
  static constexpr uint8_t kQueueDepth = 8;
  static constexpr size_t kRequestLineMax = 128;
  static constexpr size_t kSendChunk = 192;

  struct RtcmFrame {
    uint16_t len = 0;
    uint16_t sent = 0;
    uint8_t bytes[kFrameMax];
  };

  void ensureStation();
  void acceptClient();
  void readRequest(bool baseActive);
  void handleRequestLine(bool baseActive);
  void sendSourceTable();
  void rejectRequest(uint16_t status, const char *reason);
  void sendRtcmChunk();
  void closeClient();
  void clearQueue();
  void sendBeacon(unsigned long now);

  WiFiServer server_{kPort, 2};
  WiFiClient client_;
  WiFiUDP udp_;
  RtcmFrame queue_[kQueueDepth];
  uint8_t queueHead_ = 0;
  uint8_t queueCount_ = 0;
  char requestLine_[kRequestLineMax];
  size_t requestLineLen_ = 0;
  char ssid_[33] = {};
  char pass_[65] = {};
  unsigned long requestStartedMs_ = 0;
  unsigned long lastTxProgressMs_ = 0;
  unsigned long nextJoinMs_ = 0;
  unsigned long lastBeaconMs_ = 0;
  bool serverStarted_ = false;
  bool streaming_ = false;
  bool wanted_ = false;
  bool udpBound_ = false;
};
