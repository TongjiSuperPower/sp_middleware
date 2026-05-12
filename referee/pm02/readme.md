# PM02 Demo

新建applications/uart_task.cpp
```cpp
#include "cmsis_os.h"
#include "referee/pm02/pm02.hpp"

// C板
sp::PM02 pm02(&huart6);

// 达妙
// sp::PM02 pm02(&huart1, false);

extern "C" void uart_task()
{
  pm02.request();

  while (true) {
    // 使用调试(f5)查看pm02内部变量的变化
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == pm02.huart) {
    pm02.request();
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/uart_task.cpp # <- 添加这一行
    sp_middleware/referee/pm02/pm02.cpp # <- 添加这一行
    sp_middleware/tools/crc/crc.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```