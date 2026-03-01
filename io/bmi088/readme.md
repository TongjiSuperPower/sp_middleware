# BMI088 Demo

新建applications/imu_task.cpp
```c++
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// C板
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

// 达妙
// sp::BMI088 bmi088(&hspi2, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_3, r_ab);

sp::Mahony imu(1e-3f);

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
    sp_middleware/tools/gimbal/gimbal.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

# 如何确定R_ab
R_ab是旋转矩阵, 它的3列依次表示坐标系{b}的3个基向量在坐标系{a}下的坐标:

对于云台系{a}: 
- x轴正方向为子弹发射方向
- y轴正方向沿pitch轴指向云台的左侧
- z轴正方向可以根据右手螺旋确定
- 注意，只有在云台和底盘平行时, z轴才和yaw轴重合

对于bmi088系{b}, 如图1和图2所示:
![Image](https://github.com/user-attachments/assets/554dd0e0-4269-489e-b3b8-6562a2f66157)
*图1 [BMI088数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)*

![Image](https://github.com/user-attachments/assets/6faf47cc-08de-4d86-af93-679832e41167)
*图2 注意BMI088左上角的圆点*

例如, C板横着安装在云台上, 同时C板CAN一侧朝前:
```
{{0.0f, -1.0f, 0.0f}, 
 {1.0f,  0.0f, 0.0f},
 {0.0f,  0.0f, 1.0f}}
```

# Note

SPI传输速度很快, 因此只实现了阻塞读取方式. 如果采用中断, 必须要开DMA: https://community.st.com/t5/stm32-mcus-products/overrun-flag-when-using-spi-in-interrupt-mode-hal-spi-receive-it/td-p/305622


# 加热
为使陀螺仪减少温漂，我们需要将陀螺仪加热至适宜温度(dm_mc02 50°C)

在imu_task中使用pid控制温度
```c++
uint8_t first_temperate;

// -------------------- 控制参数 --------------------
//IMU温度
constexpr float IMU_TEMP = 50.0f;
//PID参数
constexpr float IMU_TEMP_KP = 400.0f;
constexpr float IMU_TEMP_KI = 0.0f;
constexpr float IMU_TEMP_KD = 0.0f;
constexpr float IMU_TEMP_MAXOUT = 1000.0f;
constexpr float IMU_TEMP_MAXIOUT = 0.0f;

sp::PID imu_temp_pid(
  1e-3, IMU_TEMP_KP, IMU_TEMP_KI, IMU_TEMP_KD, IMU_TEMP_MAXOUT, IMU_TEMP_MAXIOUT, 1.0f);

//陀螺仪温度控制函数
void imu_temp_control(float temp)
{
  uint16_t tempPWM;
  static uint8_t temp_constant_time = 0;
  if (first_temperate) {
    imu_temp_pid.calc(IMU_TEMP, bmi088.temp);
    if (imu_temp_pid.out < 0.0f) {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.out;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, tempPWM);
  }
  else {
    //在没有达到设置的温度，一直最大功率加热
    //in beginning, max power
    if (temp > IMU_TEMP) {
      temp_constant_time++;
      if (temp_constant_time > 200) {
        first_temperate = 1;
      }
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 4999);
  }
}

extern "C" void imu_task()
{
  //启动定时器10，用于IMU加热
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  while (true) {
    imu_temp_control(bmi088.temp);
    
    osDelay(1);
  }
}
```