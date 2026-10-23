# FDCAN Demo (H7板)

新建applications/fdcan_task.cpp
```cpp
#include "cmsis_os.h"
#include "io/fdcan/fdcan.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::FDCAN fdcan1(&hfdcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020

extern "C" void fdcan_task()
{
  fdcan1.start();

  while (true) {
    motor6020.cmd(0.0f);  // 调整为0.1f, 电机会旋转, 注意安全

    motor6020.write(fdcan1.tx_data);
    fdcan1.send(motor6020.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (hfdcan == &hfdcan1) {
      fdcan1.recv();

      if (fdcan1.rx_id == motor6020.rx_id) motor6020.read(fdcan1.rx_data, stamp_ms);
    }
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/fdcan_task.cpp # <- 添加这一行
    sp_middleware/io/fdcan/fdcan.cpp # <- 添加这一行
    sp_middleware/motor/rm_motor/rm_motor.cpp # <- 添加这一行
    sp_middleware/tools/math_tools/math_tools.cpp # <- 添加这一行

)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```
