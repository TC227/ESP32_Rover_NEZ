#include "Hc12Radio.h"

#include <string.h>

void Hc12Radio::begin(int8_t rxPin, int8_t txPin, int8_t setPin) {
  rxPin_ = rxPin;
  txPin_ = txPin;
  setPin_ = setPin;
  if (setPin_ >= 0) {
    pinMode(setPin_, OUTPUT);
    digitalWrite(setPin_, LOW);  // AT mode first — clones often ignore SET until it was low at power-up
  }
  uart_.setRxBufferSize(1024);
  uart_.setTxBufferSize(1024);
  uart_.begin(kAtBaud, SERIAL_8N1, rxPin_, txPin_);
  baud_ = kAtBaud;
  ready_ = false;
  atOk_ = false;
  while (uart_.available() > 0) uart_.read();
}

void Hc12Radio::enterAt() {
  if (setPin_ >= 0) digitalWrite(setPin_, LOW);
  delay(250);
  uart_.updateBaudRate(kAtBaud);
  baud_ = kAtBaud;
  delay(80);
  while (uart_.available() > 0) uart_.read();
}

void Hc12Radio::leaveAt(uint32_t baud) {
  if (setPin_ >= 0) digitalWrite(setPin_, HIGH);
  delay(250);
  uart_.updateBaudRate(baud);
  baud_ = baud;
  while (uart_.available() > 0) uart_.read();
}

bool Hc12Radio::atCommand(const char *cmd) {
  while (uart_.available() > 0) uart_.read();
  uart_.print(cmd);
  uart_.print("\r\n");
  char buf[80] = {};
  size_t n = 0;
  const unsigned long start = millis();
  while (millis() - start < 800) {
    while (uart_.available() > 0 && n + 1 < sizeof(buf)) {
      buf[n++] = static_cast<char>(uart_.read());
      buf[n] = '\0';
      if (strstr(buf, "OK") != nullptr) return true;
    }
    delay(1);
  }
  Serial.printf("HC-12 %s -> %s\n", cmd, n ? buf : "(no reply)");
  return false;
}

bool Hc12Radio::pingAt() {
  if (atCommand("AT")) return true;
  while (uart_.available() > 0) uart_.read();
  uart_.print("AT\r");
  char buf[80] = {};
  size_t n = 0;
  const unsigned long start = millis();
  while (millis() - start < 800) {
    while (uart_.available() > 0 && n + 1 < sizeof(buf)) {
      buf[n++] = static_cast<char>(uart_.read());
      buf[n] = '\0';
      if (strstr(buf, "OK") != nullptr) return true;
    }
    delay(1);
  }
  return false;
}

bool Hc12Radio::configure() {
  if (setPin_ >= 0) digitalWrite(setPin_, LOW);
  delay(200);
  // Clones keep AT at the last AT+B rate. Factory is 9600; after a successful
  // AT+B115200 the next boot only answers at 115200.
  static const uint32_t kProbeBauds[] = {115200, 9600, 57600, 38400, 19200};
  bool alive = false;
  for (uint32_t probe : kProbeBauds) {
    uart_.updateBaudRate(probe);
    baud_ = probe;
    delay(40);
    while (uart_.available() > 0) uart_.read();
    for (int i = 0; i < 2 && !alive; i++) alive = pingAt();
    if (alive) {
      Serial.printf("HC-12 AT ok at %lu\n", (unsigned long)probe);
      break;
    }
  }
  if (!alive) {
    leaveAt(kAtBaud);
    ready_ = true;
    atOk_ = false;
    Serial.println("HC-12 AT not answered at 115200 or 9600");
    return false;
  }
  char ch[12];
  snprintf(ch, sizeof(ch), "AT+C%03u", kChannel);
  atCommand(ch);
  atCommand("AT+FU3");
  atCommand("AT+P8");
  bool baudOk = false;
  for (int i = 0; i < 3 && !baudOk; i++) {
    baudOk = atCommand("AT+B115200");
    if (!baudOk) delay(80);
  }
  if (!baudOk) {
    leaveAt(kAtBaud);
    ready_ = true;
    atOk_ = false;
    Serial.println("HC-12 AT ok but AT+B115200 failed — staying 9600");
    return false;
  }
  leaveAt(kWorkBaud);
  ready_ = true;
  atOk_ = true;
  Serial.printf("HC-12 transparent %lu baud ch=%u\n", (unsigned long)baud_, kChannel);
  return true;
}

size_t Hc12Radio::write(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) return 0;
  const int room = uart_.availableForWrite();
  if (room < static_cast<int>(len)) {
    drops_++;
    return 0;
  }
  const size_t n = uart_.write(data, len);
  txBytes_ += n;
  return n;
}

size_t Hc12Radio::writeText(const char *text) {
  if (text == nullptr || !text[0]) return 0;
  return write(reinterpret_cast<const uint8_t *>(text), strlen(text));
}

size_t Hc12Radio::read(uint8_t *data, size_t cap) {
  if (data == nullptr || cap == 0) return 0;
  const int avail = uart_.available();
  if (avail <= 0) return 0;
  const size_t n = uart_.readBytes(data, avail < static_cast<int>(cap) ? avail : cap);
  rxBytes_ += n;
  return n;
}

bool Hc12Radio::pollLine(char *out, size_t cap) {
  if (out == nullptr || cap < 2) return false;
  while (uart_.available() > 0) {
    const int ch = uart_.read();
    if (ch < 0) break;
    rxBytes_++;
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (lineLen_ == 0) continue;
      line_[lineLen_] = '\0';
      strncpy(out, line_, cap - 1);
      out[cap - 1] = '\0';
      lineLen_ = 0;
      return true;
    }
    if (lineLen_ == 0 && ch != '$') continue;
    if (lineLen_ + 1 < sizeof(line_)) line_[lineLen_++] = static_cast<char>(ch);
    else lineLen_ = 0;
  }
  return false;
}
