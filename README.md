# ESP32 BMS 主项目

ESP32-C3 + ST7789 320×170 BMS 仪表完整工程。本仓库以 `JK_BMS_ESP32C3_ST7789_V6` 为代码基线，集成协议调度修复，并保留 PlatformIO 与 Arduino 两种工程入口。

## 工程入口
- PlatformIO：`platformio.ini`
- PlatformIO 主程序：`src/main.cpp`
- BLE 接收与重连：`src/ble/BmsBle.cpp`
- 协议调度：`src/protocol/BmsProtocolManager.cpp`
- JK 协议：`src/protocol/jk/JkProtocol.cpp`
- TT 协议：`src/protocol/tt/TtProtocol.cpp`
- 屏幕引脚：`src/tft_setup.h`
- 运行逻辑审计：`docs/PROTOCOL_RUNTIME_AUDIT.md`

## 本次同步修复
- 协议管理器区分候选帧协议与已确认的活动协议，避免无效帧切换后续命令所使用的协议。
- TT 增加显式变长帧长度计算。
- JK 解析保留本机配置字段，并重置本帧遥测字段，避免使用旧帧数据。
- 加入 GitHub Actions PlatformIO 自动编译工作流。

## 重要限制
JBD 已接入 DD 帧组帧/校验、0x03 基本信息、0x04 单体电压解析，并支持常见 FF00/FF01/FF02 BLE UUID；JK 兼容模块仍使用 FFE0/FFE1/FFE2。JBD 字段偏移已按常见协议表修正，但不同固件的温度/电流缩放仍需真实抓包确认。\n- DALY 当前仍需补齐并验证真实协议解析；ANT、TT、彦阳寄存器映射、CRC 和字段缩放也需要用真实设备抓包验证。CI 编译成功不等于硬件与协议行为已经验证。
