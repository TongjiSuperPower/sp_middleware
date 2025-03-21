#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tim.h"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

//温度控制相关参数
uint8_t first_temperate;
//IMU适宜温度
constexpr float IMU_TEMP = 50.0f;
//PID参数
constexpr float IMU_TEMP_KP = 1600.0f;
constexpr float IMU_TEMP_KI = 0.2f;
constexpr float IMU_TEMP_KD = 0.0f;
constexpr float IMU_TEMP_MAXOUT = 4500.0f;
constexpr float IMU_TEMP_MAXIOUT = 1000.0f;

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);  // C板, TODO 达妙
sp::Mahony imu(1e-3f);
sp::PID imu_temp_pid(
  1e-3, IMU_TEMP_KP, IMU_TEMP_KI, IMU_TEMP_KD, IMU_TEMP_MAXOUT, IMU_TEMP_MAXIOUT, 1.0f);

//让陀螺仪反馈值更平滑易于控制
sp::LowPassFilter acc_0_filter(0.5f);
sp::LowPassFilter acc_1_filter(0.5f);
sp::LowPassFilter acc_2_filter(0.5f);

sp::LowPassFilter gyro_0_filter(0.5f);
sp::LowPassFilter gyro_1_filter(0.5f);
sp::LowPassFilter gyro_2_filter(0.5f);

//零漂校准后的角速度xyz
float ins_gyro[3] = {0.0f, 0.0f, 0.0f};
//滤波后的加速度计数据
float ins_accel[3] = {0.0f, 0.0f, 0.0f};

//50℃的角速度零飘值
constexpr float GyroXZero = 0.00563896f;
constexpr float GyroYZero = 0.00107713f;
constexpr float GyroZZero = -0.0007419f;

float gyro_x_zero = GyroXZero;
float gyro_y_zero = GyroYZero;
float gyro_z_zero = GyroZZero;

void imu_temp_control(float temp);

extern "C" void imu_task()
{
  osDelay(7);
  bmi088.init();
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  while (true) {
    imu_temp_control(bmi088.temp);
    bmi088.update();
    // 使用调试(f5)查看bmi088和imu内部变量的变化
    //传感器角速度减去零飘值
    ins_gyro[0] = bmi088.gyro[0] - gyro_x_zero;  //roll
    ins_gyro[1] = bmi088.gyro[1] - gyro_y_zero;  //pitch
    ins_gyro[2] = bmi088.gyro[2] - gyro_z_zero;  //yaw

    gyro_0_filter.update(ins_gyro[0]);
    gyro_1_filter.update(ins_gyro[1]);
    gyro_2_filter.update(ins_gyro[2]);
    ins_gyro[0] = gyro_0_filter.out;
    ins_gyro[1] = gyro_1_filter.out;
    ins_gyro[2] = gyro_2_filter.out;

    acc_0_filter.update(bmi088.acc[0]);
    acc_1_filter.update(bmi088.acc[1]);
    acc_2_filter.update(bmi088.acc[2]);
    ins_accel[0] = acc_0_filter.out;
    ins_accel[1] = acc_1_filter.out;
    ins_accel[2] = acc_2_filter.out;

    osDelay(1);
    //解算四元数，欧拉角
    imu.update(ins_accel, ins_gyro);
  }
}

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
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
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
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 4999);
  }
}


```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/imu_task.cpp # <- 添加这一行
    sp_middleware/io/bmi088/bmi088.cpp # <- 添加这一行
    sp_middleware/tools/mahony/mahony.cpp # <- 添加这一行
    sp_middleware/tools/pid/pid.cpp # <- 添加这一行
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```