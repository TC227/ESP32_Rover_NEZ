#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

// Transparent SI4463 UART radio. SET low talks AT at 9600; SET high is the data pipe.
class Hc12Radio {
 public:
  static constexpr uint32_t kWorkBaud = 115200;
  static constexpr uint32_t kAtBaud = 9600;
  static constexpr uint8_t kChannel = 1;  // 433.4 MHz

  void begin(int8_t rxPin, int8_t txPin, int8_t setPin);
  bool configure();
  bool ready() const { return ready_; }
  uint32_t baud() const { return baud_; }
  size_t write(const uint8_t *data, size_t len);
  size_t writeText(const char *text);
  size_t read(uint8_t *data, size_t cap);
  bool pollLine(char *out, size_t cap);
  uint32_t txBytes() const { return txBytes_; }
  uint32_t rxBytes() const { return rxBytes_; }
  uint32_t drops() const { return drops_; }

 private:
  bool atCommand(const char *cmd);
  void enterAt();
  void leaveAt(uint32_t baud);

  HardwareSerial uart_{1};
  int8_t rxPin_ = -1;
  int8_t txPin_ = -1;
  int8_t setPin_ = -1;
  uint32_t baud_ = kAtBaud;
  bool ready_ = false;
  uint32_t txBytes_ = 0;
  uint32_t rxBytes_ = 0;
  uint32_t drops_ = 0;
  char line_[80] = {};
  size_t lineLen_ = 0;
};
