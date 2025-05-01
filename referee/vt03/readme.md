# VT03 Demo

新建applications/uart_task.cpp
```cpp
#include "cmsis_os.h"
#include "referee/vt03/vt03.hpp"

// C板
sp::vt03 vt03(&huart6);

// 达妙
// sp::vt03 vt03(&huart1, false);

extern "C" void uart_task()
{
  vt03.request();

  while (true) {
    // 使用调试(f5)查看vt03内部变量的变化
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == vt03.huart) {
    vt03.update(Size);
    vt03.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == vt03.huart) {
    vt03.request();
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/uart_task.cpp # <- 添加这一行
    sp_middleware/referee/vt03/vt03.cpp # <- 添加这一行
    sp_middleware/tools/crc/crc.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```