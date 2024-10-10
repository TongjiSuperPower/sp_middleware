# BMI088 Demo

新建applications/imu_task.cpp
```c++
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

io::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab); // C板, TODO 达妙
tools::Mahony imu(1e-3f);

extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    // 使用调试(f5)查看bmi088和imu内部变量的变化

    osDelay(1);
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/imu_task.cpp # <- 添加这一行
    sp_middleware/io/bmi088/bmi088.cpp # <- 添加这一行
    sp_middleware/tools/mahony/mahony.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

# Note

BMI088数据手册: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

SPI传输速度很快, 因此采用阻塞方式读取, 而不是中断或者DMA: https://community.st.com/t5/stm32-mcus-products/overrun-flag-when-using-spi-in-interrupt-mode-hal-spi-receive-it/td-p/305622

## 如何确定R_ab

例如, C板横着安装在云台上, 即C板的CAN口朝前:

对于云台系{a}: 
- x轴正方向为子弹发射方向,
- y轴正方向为沿pitch轴方向指向云台的左侧
- z轴正方向可以根据右手螺旋确定, 注意z轴只有在云台水平时和yaw轴重合

对于bmi088系{b}:
- x轴正方向为字母"R"的头顶所指方向
- y轴正方向为SWD烧录口朝向
- z轴正方向垂直C板向上

此时R_ab为`{{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}`