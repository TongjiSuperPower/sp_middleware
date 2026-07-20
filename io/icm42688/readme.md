# ICM42688 Demo

新建 `applications/imu_task.cpp`：
```c++
#include "cmsis_os.h"
#include "io/icm42688/icm42688.hpp"
#include "tools/mahony/mahony.hpp"

// 旋转矩阵配置 (请根据实际安装方向调整)
const float r_ab[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// 实例化 ICM42688 

sp::ICM42688 icm42688(&hspi1, GPIOA, GPIO_PIN_4, r_ab);

sp::Mahony imu(1e-3f); // 1e-3f 代表算法 1000Hz

extern "C" void imu_task()
{
  icm42688.init();

  while (true) {
    icm42688.update();
    imu.update(icm42688.acc, icm42688.gyro);

    // 1kHz / 1000Hz 的频率调度
    // osDelay(1) 将让任务每次休眠 1ms 从而实现严格的 1000Hz 刷新率
    osDelay(1);
  }
}