#ifndef IO__INS_HPP_
#define IO__INS_HPP_
extern "C" {
#include "BMI088Middleware.h"
#include "BMI088driver.h"
}
#include <array>

#include "cmsis_os.h"
#include "spi.h"

namespace io
{
static constexpr uint8_t kSpiDmaGyroLength = 8;       // Gyro DMA 缓冲区长度
static constexpr uint8_t kSpiDmaAccelLength = 9;      // Accel DMA 缓冲区长度
static constexpr uint8_t kSpiDmaAccelTempLength = 4;  // Gyro Temp DMA 缓冲区长度
static constexpr float filter_num[3] = {
  1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};  // 滤波器系数
static constexpr uint16_t MPU6500_TEMP_PWM_MAX = 4999;  //mpu6500控制温度的设置TIM的重载值
static constexpr int BMI088_GYRO_RX_BUF_DATA_OFFSET = 1;
static constexpr int BMI088_ACCEL_RX_BUF_DATA_OFFSET = 2;
static constexpr int IMU_DR_SHFITS = 0;
static constexpr int IMU_SPI_SHFITS = 1;
static constexpr int IMU_UPDATE_SHIFTS = 2;
static constexpr int IMU_NOTIFY_SHIFTS = 3;
static constexpr float SetTemp = 45.0f;  // 设定温度值

// AHRS相关参数
constexpr float sampleFreq = 1000.0f;    // 采样频率，单位 Hz
constexpr float twoKpDef = 2.0f * 0.5f;  // 比例增益的默认值
constexpr float twoKiDef = 2.0f * 0.0f;  // 积分增益的默认值

class Ins
{
public:
  Ins(
    SPI_HandleTypeDef & hspi, SPI_TypeDef * SPIx, TIM_HandleTypeDef * htim, uint16_t ExInterruptGpio,
    bmi088_real_data_t & bmi088_real_data, DMA_HandleTypeDef & hdma_spi_rx,
    DMA_HandleTypeDef & hdma_spi_tx, const std::array<std::array<float, 3>, 3> & rotMatrix);
  void Init();
  void Update();
  void gpioCallback(uint16_t GPIO_Pin, TaskHandle_t & INS_task_local_handler);
  void dmaIrqHandler(void);
  void ahrsUpdate();
  void quat2Euler();
  const float * getEulerAngle() const;
  const float * getQuaternion() const;
  const float * getGyro() const;
  const float * getAccel() const;

private:
  SPI_HandleTypeDef & hspi_;
  SPI_TypeDef * SPIx_;
  TIM_HandleTypeDef * htim_;
  uint16_t ExInterruptGpio_;
  bmi088_real_data_t & bmi088_real_data_;
  DMA_HandleTypeDef & hdma_spi_rx_;
  DMA_HandleTypeDef & hdma_spi_tx_;
  const std::array<std::array<float, 3>, 3> & rotMatrix_;

  float gyro_[3] = {0.0f, 0.0f, 0.0f};        // 陀螺仪数据
  float accel_[3] = {0.0f, 0.0f, 0.0f};       // 加速度计数据
  float quat_[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 四元数 w x y z
  float angle_[3] = {0.0f, 0.0f, 0.0f};

  float accel_filter_1_[3] = {0.0f, 0.0f, 0.0f};
  float accel_filter_2_[3] = {0.0f, 0.0f, 0.0f};
  float accel_filter_3_[3] = {0.0f, 0.0f, 0.0f};

  volatile uint8_t gyro_update_flag = 0;
  volatile uint8_t accel_update_flag = 0;
  volatile uint8_t accel_temp_update_flag = 0;
  volatile uint8_t imu_start_dma_flag = 0;

  uint8_t gyro_dma_tx_buf_[kSpiDmaGyroLength] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t gyro_dma_rx_buf_[kSpiDmaGyroLength];

  uint8_t accel_dma_rx_buf_[kSpiDmaAccelLength];
  uint8_t accel_dma_tx_buf_[kSpiDmaAccelLength] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF,
                                                   0xFF, 0xFF, 0xFF, 0xFF};

  uint8_t accel_temp_dma_rx_buf_[kSpiDmaAccelTempLength];
  uint8_t accel_temp_dma_tx_buf_[kSpiDmaAccelTempLength] = {0xA2, 0xFF, 0xFF, 0xFF};

  // 温度控制相关
  uint8_t first_temperature_flag = 0;

  // AHRS相关参数
  float twoKp = twoKpDef;                                            // 比例增益
  float twoKi = twoKiDef;                                            // 积分增益
  float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // 积分反馈

  void setup_spi();
  void init_spi_dma(uint32_t tx_buf, uint32_t rx_buf, uint16_t length);
  void init_dma_channel(
    DMA_HandleTypeDef & dma_handle, uint32_t buffer, uint16_t length, uint32_t clear_flag);

  void apply_rotmatrix(const float input[3], float output[3]);
  void imu_cali_solve();

  void imu_pwm_set(uint16_t pwm);     // 设定 PWM 以控制温度
  void imu_temp_control(float temp);  // 温度控制
  void low_pass_filter(int irrit_i);  // 低通滤波
  void imu_cmd_spi_dma(void);
  void spi_dma_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);
};

}  // namespace io

#endif
