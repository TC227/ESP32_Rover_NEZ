#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

class RoverBaseLink {
 public:
  void setCredentials(const char *ssid, const char *pass);
  void setBaseHost(const char *ip);
  bool hasCredentials() const { return ssid_[0] != '\0'; }
  bool wifiUp() const { return WiFi.status() == WL_CONNECTED; }
  const char *baseHost() const { return host_[0] ? host_ : "0.0.0.0"; }
  void begin();
  void end();
  void service(unsigned long now);
  size_t read(uint8_t *buffer, size_t capacity);
  bool streaming() const;
  bool stopRemoteBase();

 private:
  enum class State : uint8_t {
    Disabled,
    WaitingWifi,
    WaitingCaster,
    ReadingHeaders,
    Streaming
  };

  void ensureStation(unsigned long now);
  void pollBeacon();
  void startCasterConnection(unsigned long now);
  void rejectCasterConnection(unsigned long now);

  WiFiClient client_;
  WiFiUDP udp_;
  State state_ = State::Disabled;
  bool enabled_ = false;
  bool udpBound_ = false;
  char ssid_[33] = {};
  char pass_[65] = {};
  char host_[16] = {};
  char responseHeaders_[160] = {};
  size_t responseHeaderLen_ = 0;
  unsigned long lastWifiAttemptMs_ = 0;
  unsigned long nextCasterAttemptMs_ = 0;
  unsigned long responseStartedMs_ = 0;
};
