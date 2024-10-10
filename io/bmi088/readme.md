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

# 如何确定R_ab

例如, C板横着安装在云台上, 同时C板CAN一侧朝前:

对于云台系{a}: 
- x轴正方向为子弹发射方向
- y轴正方向沿pitch轴指向云台的左侧
- z轴正方向可以根据右手螺旋确定
- 注意，只有在云台和底盘平行时, z轴才和yaw轴重合

对于bmi088系{b}:
- x轴正方向为C板上字母"R"的头顶一侧
- y轴正方向指向C板SWD一侧
- z轴正方向垂直C板顶面朝上, 同样符合右手螺旋

R_ab是旋转矩阵, 它的3列依次表示坐标系{b}的3个基向量在坐标系{a}下的坐标:

`{{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}`

# Note

BMI088数据手册: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

SPI传输速度很快, 因此采用阻塞方式读取, 而不是中断或者DMA: https://community.st.com/t5/stm32-mcus-products/overrun-flag-when-using-spi-in-interrupt-mode-hal-spi-receive-it/td-p/305622
