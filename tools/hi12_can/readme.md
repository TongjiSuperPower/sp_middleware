# HI12 CAN Demo（H7板）

适用于本项目的imu：
- 型号：HI12S3-MI1-000
- 经典 CAN  1 Mbit/s
- 节点地址 `0x08`
- 连接到 CAN3。

当前类固定使用 `hfdcan3` 和节点地址 `0x08`，接收四元数和 Yaw。


## 使用前配置

- CubeMX 配置
  
| CAN3参数                 | 设置                |
| ------------------------ | ------------------- |
| Nominal Prescaler        | 3                   |
| Nominal Sync Jump Width  | 1                   |
| Nominal Time Seg1 / Seg2 | 29 / 10             |
| Auto Retransmission      | Enable              |
| Frame Format             | Classic CAN         |
| Standard Filters Number  | 14                  |
| Extended Filters Number  | 1                   |
| RX FIFO0                 | 4个元素，每个64字节 |
| RX FIFO1                 | 4个元素，每个64字节 |
| TX FIFO                  | 4个元素，每个64字节 |
| Message RAM Offset       | 0x812               |

- 在模块端开启下面两个 PGN 的周期输出，例如周期均为 10 ms（各 100 Hz）。

| 数据   | PGN      | 节点 8 的 CAN ID | 周期参数地址 |
| ------ | -------- | ---------------- | ------------ |
| 四元数 | `0xFF46` | `0x0CFF4608`     | `0x0146`     |
| Yaw    | `0xFF41` | `0x0CFF4108`     | `0x0141`     |


## 编辑 applications/can_task.cpp

```cpp
#include "cmsis_os.h"
#include "io/fdcan/fdcan.hpp"
#include "tools/hi12_can/hi12_can.hpp"

sp::FDCAN fdcan3(&hfdcan3);
sp::HI12Can hi12;  // 应用层实例化

extern "C" void can_task(void const *)
{
  hi12.prepare();  // 初始化过滤器和 FIFO1 通知，必须在启动 CAN3 前调用
  fdcan3.start();

  while (true) {
    osDelay(1);
  }
}

// 标准帧电机继续使用 FIFO0。
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t)
{
  if (hfdcan != &hfdcan3) return;
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    fdcan3.recv();
    // 在这里按电机 ID 分流，并调用对应电机的 read()。
  }
}

// HI12 扩展帧使用 FIFO1。
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t)
{
  if (hfdcan != &hfdcan3) return;
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[64];
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, data) != HAL_OK) return;
    hi12.read(header, data, HAL_GetTick());
  }
}
```

## 编辑 CMakeLists.txt

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/can_task.cpp
    sp_middleware/io/fdcan/fdcan.cpp
    sp_middleware/tools/hi12_can/hi12_can.cpp
    sp_middleware/tools/hi12_can/hi12_sdk/hipnuc_j1939.c
    sp_middleware/tools/hi12_can/hi12_sdk/hipnuc_sample.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/
)
```