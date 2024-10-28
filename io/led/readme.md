# LED Demo (C板)

新建applications/led_task.cpp
```cpp
#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task()
{
  led.start();

  while (true) {
    for (uint8_t g = 0; g < 10; g++) {
      led.set(0, g * 0.01f, 0);
      osDelay(100);
    }

    for (uint8_t g = 10; g > 0; g--) {
      led.set(0, g * 0.01f, 0);
      osDelay(100);
    }
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/led_task.cpp # <- 添加这一行
    sp_middleware/io/led/led.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```