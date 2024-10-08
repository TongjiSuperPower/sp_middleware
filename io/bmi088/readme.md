# BMI088 Demo

新建applications/imu_task.cpp
```c++
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"

// C板
io::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0);

// TODO 达妙

extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();

    // 使用调试(f5)查看bmi088内部变量的变化

    osDelay(1);
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/imu_task.cpp # <- 添加这一行
    sp_middleware/io/bmi088/bmi088.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

# Note

BMI088数据手册: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

SPI传输速度很快, 因此采用阻塞方式读取, 而不是中断或者DMA: https://community.st.com/t5/stm32-mcus-products/overrun-flag-when-using-spi-in-interrupt-mode-hal-spi-receive-it/td-p/305622