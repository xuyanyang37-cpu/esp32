#ifndef JBD_PROTOCOL_H
#define JBD_PROTOCOL_H

#include <Arduino.h>
#include "../BmsProtocol.h"

// 嘉佰达 JBD 常见 DD 帧协议；不同固件/蓝牙模块需用真实抓包验证。
class JbdProtocol : public BmsProtocol {
public:
  const char* name() const override { return "JBD"; }
  bool canHandle(const uint8_t*, size_t) const override;
  int findFrameStart(const uint8_t*, size_t) const override;
  bool parseFrame(const uint8_t*, size_t, BmsData&) override;
  bool buildCommand(uint8_t, uint8_t, uint8_t[20]) override;
  size_t expectedFrameLength() const override { return frameLength_; }
  size_t frameLength(const uint8_t*, size_t) const override;
private:
  mutable size_t frameLength_ = 0;
  static uint16_t u16be(const uint8_t*);
  static int16_t s16be(const uint8_t*);
  static bool validFrame(const uint8_t*, size_t);
  static uint16_t checksum(const uint8_t*, size_t);
};
#endif
