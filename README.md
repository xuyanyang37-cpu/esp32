# ESP32 BMS 主项目

本仓库用于 ESP32-C3 + ST7789 BMS 仪表固件。工程基线同步自 JK_BMS_ESP32C3_ST7789_V6，并包含协议调度修复与运行逻辑审计文档。

- PlatformIO 工程：`platformio.ini`
- 主程序：`src/main.cpp`
- BLE 接收：`src/ble/BmsBle.cpp`
- 协议管理：`src/protocol/BmsProtocolManager.cpp`
- 屏幕引脚：`src/tft_setup.h`

注意：JBD/DALY 解析器目前仍是占位实现；实际协议、寄存器和硬件行为需要用真实 BMS 数据验证。
