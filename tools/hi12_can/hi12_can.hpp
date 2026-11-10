#ifndef SP_HI12_CAN_HPP
#define SP_HI12_CAN_HPP

#include <cstdint>

#include "fdcan.h"

namespace sp
{
class HI12Can
{
public:
  volatile float quat[4] = {};  // 四元数，顺序为 w、x、y、z，无单位。
  volatile float yaw = 0;       // 偏航角，单位 rad，逆时针为正。
  volatile uint32_t stamp_ms[2] = {};  // 最近接收时间（毫秒）：四元数、Yaw。

  // CAN3 初始化后、启动前调用一次；标准帧电机仍使用 FIFO0。
  bool prepare();
  // 由 FIFO1 回调传入接收到的帧，更新成员变量。
  void read(const FDCAN_RxHeaderTypeDef & header, const uint8_t * data, uint32_t stamp);
};

}  // namespace sp
#endif
