#ifndef JK02_32S_H
#define JK02_32S_H
#include <Arduino.h>
class Jk02_32S {
public:
  static const uint8_t MAX_CELLS=32;
  static const int DATA_OFFSET=32; // 主数据区偏移：16字节电芯区扩展在后续结构中折算为32字节
  static const int CELL_MASK_OFFSET=70; // 32个电芯电压(6..69)后紧接使能电芯bitmask
  static const size_t FRAME_LENGTH=300;
  static const char* name(){return "JK02_32S";}
};
#endif
