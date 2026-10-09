#include "AntProtocol.h"
#include <string.h>

// ANT 2021+ BLE 实时状态帧：7E A1 11 ... CRC_LO CRC_HI AA 55
// CRC-16/Modbus：初值 FFFF，多项式 A001，校验范围从 A1 到数据区末尾。
uint16_t AntProtocol::crc16(const uint8_t* data,size_t len){
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<len;i++){
    crc^=data[i];
    for(uint8_t b=0;b<8;b++)
      crc=(crc&1)?uint16_t((crc>>1)^0xA001):uint16_t(crc>>1);
  }
  return crc;
}

uint16_t AntProtocol::u16le(const uint8_t* p){
  return uint16_t(p[0]) | (uint16_t(p[1])<<8);
}

uint32_t AntProtocol::u32le(const uint8_t* p){
  return uint32_t(p[0]) | (uint32_t(p[1])<<8) |
         (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24);
}

int16_t AntProtocol::s16le(const uint8_t* p){
  return int16_t(u16le(p));
}

bool AntProtocol::validFrame(const uint8_t* p,size_t n){
  if(!p || n<10) return false;
  if(p[0]!=0x7E || p[1]!=0xA1 || p[2]!=0x11) return false;

  // Byte 5 是数据长度；整帧 = 6 字节头部 + 数据 + CRC(2) + 帧尾(2)。
  const size_t total=size_t(p[5])+10U;
  if(total!=n) return false;
  if(p[total-2]!=0xAA || p[total-1]!=0x55) return false;

  const uint16_t calc=crc16(p+1,total-5U);
  const uint16_t recv=uint16_t(p[total-4]) | (uint16_t(p[total-3])<<8);
  return calc==recv;
}

bool AntProtocol::canHandle(const uint8_t* p,size_t n) const{
  return p && n>=3 && p[0]==0x7E && p[1]==0xA1 && p[2]==0x11;
}

int AntProtocol::findFrameStart(const uint8_t* p,size_t n) const{
  if(!p || n<2) return -1;
  for(size_t i=0;i+1<n;i++)
    if(p[i]==0x7E && p[i+1]==0xA1) return int(i);
  return -1;
}

size_t AntProtocol::frameLength(const uint8_t* p,size_t n) const{
  if(!p || n<6 || p[0]!=0x7E || p[1]!=0xA1) return 0;
  // 本解析器只处理状态应答 0x11；其他响应不能按实时数据结构解释。
  if(p[2]!=0x11) return 0;
  const size_t total=size_t(p[5])+10U;
  // p[5] 为单字节，合法整帧长度最多 265 字节。
  if(total<10U || total>265U) return 0;
  return total;
}

bool AntProtocol::parseFrame(const uint8_t* p,size_t n,BmsData& o){
  if(!validFrame(p,n)) return false;

  const uint8_t tempSensors=p[8];
  const uint8_t cells=p[9];
  if(tempSensors>6 || cells==0 || cells>JK_MAX_CELLS) return false;

  // 帧偏移说明：
  // p[6..33] = 固定状态头28字节；电芯数据从 p[34] 开始。
  // 动态段为 cells*2 + tempSensors*2；随后读取到动态段偏移+51。
  const size_t dynamicBytes=size_t(cells)*2U+size_t(tempSensors)*2U;
  const size_t dynamicBase=34U+dynamicBytes;
  // 最后读取的字段是 dynamicBase+50 的 uint16，需保证两字节都在帧内。
  if(dynamicBase+52U>n-4U) return false;

  BmsData d=o;
  // 每次完整状态帧都清空遥测缓存，避免电芯数量/温度传感器减少后残留旧值。
  d.cellCount=cells;
  for(uint8_t i=0;i<JK_MAX_CELLS;i++) d.cellVoltage[i]=0.0f;
  d.temperature1=0.0f;
  d.temperature2=0.0f;
  d.mosTemperature=0.0f;
  d.totalVoltage=0.0f;
  d.current=0.0f;
  d.power=0.0f;
  d.soc=0.0f;
  d.totalCapacityAh=0.0f;
  d.remainingCapacityAh=0.0f;
  d.remainingRangeKm=0.0f;
  d.charging=false;
  d.discharging=false;
  d.balancing=false;
  d.minCellVoltage=0.0f;
  d.maxCellVoltage=0.0f;
  d.deltaCellVoltage=0.0f;
  d.minCell=0;
  d.maxCell=0;

  for(uint8_t i=0;i<cells;i++)
    d.cellVoltage[i]=u16le(p+34U+size_t(i)*2U)*0.001f;

  const size_t tempBase=34U+size_t(cells)*2U;
  if(tempSensors>=1) d.temperature1=float(s16le(p+tempBase)); // ANT 状态帧温度单位为°C
  if(tempSensors>=2) d.temperature2=float(s16le(p+tempBase+2U));
  d.mosTemperature=float(s16le(p+dynamicBase));
  d.totalVoltage=float(u16le(p+dynamicBase+4U))*0.01f;
  d.current=float(s16le(p+dynamicBase+6U))*0.1f;
  d.soc=float(u16le(p+dynamicBase+8U));

  // 状态字节：MOSFET 状态 1 表示开启；均衡状态 4 表示自动均衡。
  d.charging=(p[dynamicBase+12U]==0x01);
  d.discharging=(p[dynamicBase+13U]==0x01);
  d.balancing=(p[dynamicBase+14U]!=0x00);

  // 容量单位为 µAh，换算 Ah 需要除以 1,000,000。
  d.totalCapacityAh=float(u32le(p+dynamicBase+16U))*0.000001f;
  d.remainingCapacityAh=float(u32le(p+dynamicBase+20U))*0.000001f;

  if(d.soc>100.0f) return false;
  if(d.current>-0.05f && d.current<0.05f) d.current=0.0f;

  float minV=100.0f,maxV=0.0f;
  uint8_t minC=0,maxC=0;
  for(uint8_t i=0;i<cells;i++){
    const float v=d.cellVoltage[i];
    if(v>0.5f && v<6.0f){
      if(v<minV){minV=v;minC=i+1;}
      if(v>maxV){maxV=v;maxC=i+1;}
    } else {
      // 已声明的电芯却给出不合理电压，拒绝整帧，避免显示旧数据/垃圾值。
      return false;
    }
  }
  if(minV>=100.0f || d.totalVoltage<1.0f || d.totalVoltage>100.0f) return false;

  d.minCellVoltage=minV;
  d.maxCellVoltage=maxV;
  d.deltaCellVoltage=maxV-minV;
  d.minCell=minC;
  d.maxCell=maxC;
  d.power=d.totalVoltage*d.current;
  if(d.energyConsumptionWhKm>1.0f)
    d.remainingRangeKm=(d.remainingCapacityAh*d.totalVoltage)/d.energyConsumptionWhKm;

  d.deviceName="ANT BMS";
  d.valid=true;
  d.online=true;
  d.updateMs=millis();

  o=d;
  return true;
}

bool AntProtocol::buildCommand(uint8_t,uint8_t,uint8_t[20]){
  // ANT 查询是 10 字节帧，由 BmsBle::requestAntStatus() 构造并发送。
  // 通用 buildCommand() 缓冲区接口未携带实际长度，避免伪造 20 字节 ANT 帧。
  return false;
}
