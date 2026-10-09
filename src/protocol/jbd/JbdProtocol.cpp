#include "JbdProtocol.h"
#include <string.h>

uint16_t JbdProtocol::u16be(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

int16_t JbdProtocol::s16be(const uint8_t* p) {
  return static_cast<int16_t>(u16be(p));
}

// JBD 响应帧校验字段为 DATA 前全部字节（CMD 到 DATA）的二补数。
uint16_t JbdProtocol::checksum(const uint8_t* p, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum = uint16_t(sum + p[i]);
  return uint16_t(0U - sum);
}

int JbdProtocol::findFrameStart(const uint8_t* p, size_t n) const {
  if (!p || n < 2) return -1;
  for (size_t i = 0; i + 1 < n; ++i) {
    if (p[i] == 0xDD && p[i + 1] != 0xA5 && p[i + 1] != 0x5A)
      return int(i);
  }
  return -1;
}

size_t JbdProtocol::frameLength(const uint8_t* p, size_t n) const {
  if (!p || n < 4 || p[0] != 0xDD) return 0;
  // 响应：DD CMD STATUS LEN DATA CHK_H CHK_L 77
  size_t total = size_t(p[3]) + 7U;
  if (total < 7U || total > 255U) return 0;
  return total;
}

bool JbdProtocol::canHandle(const uint8_t* p, size_t n) const {
  if (!p || n < 2 || p[0] != 0xDD) return false;
  // 过滤常见请求帧头 DD A5 和应答帧尾标记以外的噪声。
  if (p[1] == 0xA5 || p[1] == 0x5A) return false;
  return true;
}

bool JbdProtocol::validFrame(const uint8_t* p, size_t n) {
  if (!p || n < 7 || p[0] != 0xDD || p[n - 1] != 0x77) return false;
  if (size_t(p[3]) + 7U != n) return false;
  uint16_t sum = 0;
  for (size_t i = 1; i < n - 3; ++i) sum = uint16_t(sum + p[i]);
  const uint16_t expected = uint16_t(0U - sum);
  const uint16_t received = (uint16_t(p[n - 3]) << 8) | p[n - 2];
  return expected == received;
}

bool JbdProtocol::parseFrame(const uint8_t* p, size_t n, BmsData& out) {
  if (!validFrame(p, n) || p[2] != 0x00) return false;

  const uint8_t command = p[1];
  const uint8_t dataLen = p[3];
  const uint8_t* d = p + 4;
  BmsData next = out;

  if (command == 0x03) {
    // JBD 常见基本信息响应，字段偏移针对标准 0x03 数据区。
    // 至少需要电压、电流、容量、SOC、电芯数和温度数等字段。
    if (dataLen < 23) return false;

    next.totalVoltage = float(u16be(d + 0)) * 0.01f;
    next.current = float(s16be(d + 2)) * 0.01f;
    next.remainingCapacityAh = float(u16be(d + 4)) * 0.01f;
    next.totalCapacityAh = float(u16be(d + 6)) * 0.01f;
    next.cycleCount = u16be(d + 8);
    next.soc = float(d[19]);
    if (next.soc > 100.0f) next.soc = 100.0f;

    uint8_t cells = d[21];
    uint8_t sensors = d[22];
    if (cells == 0 || cells > JK_MAX_CELLS) return false;
    if (size_t(23U + size_t(sensors) * 2U) > dataLen) return false;

    next.cellCount = cells;
    next.errors = u16be(d + 16);
    next.charging = (d[20] & 0x01) != 0;
    next.discharging = (d[20] & 0x02) != 0;

    // 温度字段是 0.1K，转换为摄氏度。最多映射两个温度传感器。
    if (sensors >= 1) next.temperature1 = float(u16be(d + 23)) * 0.1f - 273.15f;
    if (sensors >= 2) next.temperature2 = float(u16be(d + 25)) * 0.1f - 273.15f;
    if (sensors == 0) {
      next.temperature1 = 0.0f;
      next.temperature2 = 0.0f;
    }

    next.power = next.totalVoltage * next.current;
    if (next.totalCapacityAh > 0.0f)
      next.remainingCapacityAh = min(next.remainingCapacityAh, next.totalCapacityAh);
    if (next.energyConsumptionWhKm > 1.0f && next.totalVoltage > 0.1f)
      next.remainingRangeKm = next.remainingCapacityAh * next.totalVoltage / next.energyConsumptionWhKm;
    else
      next.remainingRangeKm = 0.0f;

    next.deviceName = "JBD BMS";
    next.valid = next.totalVoltage > 0.5f && next.cellCount > 0;
  } else if (command == 0x04) {
    // 单体电压响应：每节两个字节，单位 mV；允许此帧先于基本信息到达。
    if (dataLen < 2 || (dataLen % 2) != 0) return false;
    uint8_t cells = uint8_t(dataLen / 2U);
    if (cells == 0 || cells > JK_MAX_CELLS) return false;
    next.cellCount = cells;
    float minV = 100.0f, maxV = 0.0f;
    uint8_t minCell = 0, maxCell = 0;
    for (uint8_t i = 0; i < cells; ++i) {
      float v = float(u16be(d + size_t(i) * 2U)) * 0.001f;
      if (v < 0.5f || v > 6.0f) return false;
      next.cellVoltage[i] = v;
      if (v < minV) { minV = v; minCell = uint8_t(i + 1); }
      if (v > maxV) { maxV = v; maxCell = uint8_t(i + 1); }
    }
    for (uint8_t i = cells; i < JK_MAX_CELLS; ++i) next.cellVoltage[i] = 0.0f;
    next.minCellVoltage = minV < 100.0f ? minV : 0.0f;
    next.maxCellVoltage = maxV;
    next.deltaCellVoltage = (maxV > 0.0f && minV < 100.0f) ? maxV - minV : 0.0f;
    next.minCell = minCell;
    next.maxCell = maxCell;
    if (next.totalVoltage <= 0.5f) {
      float sum = 0.0f;
      for (uint8_t i = 0; i < cells; ++i) sum += next.cellVoltage[i];
      next.totalVoltage = sum;
    }
    next.power = next.totalVoltage * next.current;
    if (next.totalVoltage > 0.5f) next.valid = true;
  } else {
    // 未支持的命令应答不切换协议、不覆盖遥测数据。
    return false;
  }

  next.online = next.valid;
  next.updateMs = millis();
  if (next.valid) out = next;
  return next.valid;
}

bool JbdProtocol::buildCommand(uint8_t command, uint8_t, uint8_t out[20]) {
  if (!out || (command != 0x03 && command != 0x04)) return false;
  memset(out, 0, 20);
  // JBD 查询请求：DD A5 CMD 00 CHK_H CHK_L 77
  out[0] = 0xDD;
  out[1] = 0xA5;
  out[2] = command;
  out[3] = 0x00;
  const uint16_t check = uint16_t(0U - command);
  out[4] = uint8_t(check >> 8);
  out[5] = uint8_t(check);
  out[6] = 0x77;
  return true;
}
