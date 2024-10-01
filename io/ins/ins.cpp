#include "ins.hpp"

#include "tools/pid/pid.hpp"

tools::PID imu_temp_pid(1e-3f, 1600.0f, 0.2f, 0.0f, 4500.0f, 4400.0f);

namespace io
{
Ins::Ins(
  SPI_HandleTypeDef & hspi, SPI_TypeDef * SPIx, TIM_HandleTypeDef * htim, uint16_t ExInterruptGpio,
  bmi088_real_data_t & bmi088_real_data, DMA_HandleTypeDef & hdma_spi1_rx,
  DMA_HandleTypeDef & hdma_spi1_tx, const std::array<std::array<float, 3>, 3> & rotMatrix)
: hspi_(hspi),
  SPIx_(SPIx), //SPI_BASE地址
  htim_(htim),
  ExInterruptGpio_(ExInterruptGpio),
  bmi088_real_data_(bmi088_real_data),
  hdma_spi_rx_(hdma_spi1_rx),
  hdma_spi_tx_(hdma_spi1_tx),
  rotMatrix_(rotMatrix)
{
  // delay_init();
}

void Ins::Init()
{
  while (BMI088_init()) {
    osDelay(100);
  }
  BMI088_read(bmi088_real_data_.gyro, bmi088_real_data_.accel, &bmi088_real_data_.temp);

  imu_cali_solve();

  for (int i = 0; i < 3; ++i) {
    accel_filter_1_[i] = accel_filter_2_[i] = accel_filter_3_[i] = accel_[i];
  }
  setup_spi();
  imu_start_dma_flag = 1;
}

// 更新函数
void Ins::Update()
{
  if (gyro_update_flag & (1 << IMU_NOTIFY_SHIFTS)) {
    gyro_update_flag &= ~(1 << IMU_NOTIFY_SHIFTS);
    BMI088_gyro_read_over(
      gyro_dma_rx_buf_ + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data_.gyro);  // 读取陀螺仪数据
  }

  if (accel_update_flag & (1 << IMU_UPDATE_SHIFTS)) {
    accel_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
    BMI088_accel_read_over(
      accel_dma_rx_buf_ + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data_.accel,
      &bmi088_real_data_.time);  // 读取加速度计数据
  }

  if (accel_temp_update_flag & (1 << IMU_UPDATE_SHIFTS)) {
    accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
    BMI088_temperature_read_over(
      accel_temp_dma_rx_buf_ + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
      &bmi088_real_data_.temp);                // 读取温度数据
    imu_temp_control(bmi088_real_data_.temp);  // 温度控制
  }

  imu_cali_solve();

  // 对加速度计进行低通滤波
  for (int i = 0; i < 3; i++) low_pass_filter(i);

  taskENTER_CRITICAL();
  ahrsUpdate();
  taskEXIT_CRITICAL();
}

const float * Ins::getEulerAngle() const
{
  return angle_;  //欧拉角
}
const float * Ins::getQuaternion() const
{
  return quat_;  //四元数
}
const float * Ins::getAccel() const
{
  return accel_;  //加速度
}
const float * Ins::getGyro() const
{
  return gyro_;  //角速度
}

void Ins::ahrsUpdate()
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  const float invSampleFreq = 1.0f / sampleFreq;
  const float halfT = 0.5f * invSampleFreq;

  if (!((accel_filter_3_[0] == 0.0f) && (accel_filter_3_[1] == 0.0f) &&
        (accel_filter_3_[2] == 0.0f))) {
    recipNorm =
      1.0f / std::sqrt(
               accel_filter_3_[0] * accel_filter_3_[0] + accel_filter_3_[1] * accel_filter_3_[1] +
               accel_filter_3_[2] * accel_filter_3_[2]);
    accel_filter_3_[0] *= recipNorm;
    accel_filter_3_[1] *= recipNorm;
    accel_filter_3_[2] *= recipNorm;
    halfvx = quat_[1] * quat_[3] - quat_[0] * quat_[2];
    halfvy = quat_[0] * quat_[1] + quat_[2] * quat_[3];
    halfvz = quat_[0] * quat_[0] - 0.5f + quat_[3] * quat_[3];
    halfex = (accel_filter_3_[1] * halfvz - accel_filter_3_[2] * halfvy);
    halfey = (accel_filter_3_[2] * halfvx - accel_filter_3_[0] * halfvz);
    halfez = (accel_filter_3_[0] * halfvy - accel_filter_3_[1] * halfvx);
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * invSampleFreq;
      integralFBy += twoKi * halfey * invSampleFreq;
      integralFBz += twoKi * halfez * invSampleFreq;
      gyro_[0] += integralFBx;
      gyro_[1] += integralFBy;
      gyro_[2] += integralFBz;
    }
    else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    gyro_[0] += twoKp * halfex;
    gyro_[1] += twoKp * halfey;
    gyro_[2] += twoKp * halfez;
  }
  gyro_[0] *= halfT;
  gyro_[1] *= halfT;
  gyro_[2] *= halfT;
  qa = quat_[0];
  qb = quat_[1];
  qc = quat_[2];
  quat_[0] += (-qb * gyro_[0] - qc * gyro_[1] - quat_[3] * gyro_[2]);
  quat_[1] += (qa * gyro_[0] + qc * gyro_[2] - quat_[3] * gyro_[1]);
  quat_[2] += (qa * gyro_[1] - qb * gyro_[2] + quat_[3] * gyro_[0]);
  quat_[3] += (qa * gyro_[2] + qb * gyro_[1] - qc * gyro_[0]);

  recipNorm =
    1.0f / std::sqrt(
             quat_[0] * quat_[0] + quat_[1] * quat_[1] + quat_[2] * quat_[2] + quat_[3] * quat_[3]);
  quat_[0] *= recipNorm;
  quat_[1] *= recipNorm;
  quat_[2] *= recipNorm;
  quat_[3] *= recipNorm;
}

void Ins::quat2Euler()
{
  angle_[0] = atan2f(
    2.0f * (quat_[0] * quat_[3] + quat_[1] * quat_[2]),
    2.0f * (quat_[0] * quat_[0] + quat_[1] * quat_[1]) - 1.0f);
  angle_[1] = asinf(-2.0f * (quat_[1] * quat_[3] - quat_[0] * quat_[2]));
  angle_[2] = atan2f(
    2.0f * (quat_[0] * quat_[1] + quat_[2] * quat_[3]),
    2.0f * (quat_[0] * quat_[0] + quat_[3] * quat_[3]) - 1.0f);
}

void Ins::imu_pwm_set(uint16_t pwm) { __HAL_TIM_SetCompare(htim_, TIM_CHANNEL_1, pwm); }

void Ins::imu_temp_control(float temp)
{
  uint16_t temp_pwm;
  static uint8_t temp_constant_time = 0;

  if (first_temperature_flag) {
    imu_temp_pid.calc(temp, SetTemp);
    imu_temp_pid.out = (imu_temp_pid.out < 0.0f) ? 0.0f : imu_temp_pid.out;  // 确保输出不会为负
    temp_pwm = static_cast<uint16_t>(imu_temp_pid.out);
    imu_pwm_set(temp_pwm);  // 设置 PWM
  }
  else {
    // 初次达到设定温度时，最大功率加热
    if (temp > SetTemp) {
      temp_constant_time++;
      if (temp_constant_time > 200) {
        first_temperature_flag = 1;
        imu_temp_pid.data.iout = MPU6500_TEMP_PWM_MAX / 2.0f;
      }
    }
    imu_pwm_set(MPU6500_TEMP_PWM_MAX - 1);  // 最大功率加热
  }
}

// 低通滤波
void Ins::low_pass_filter(int irrit_i)
{
  accel_filter_1_[irrit_i] = accel_filter_2_[irrit_i];
  accel_filter_2_[irrit_i] = accel_filter_3_[irrit_i];
  accel_filter_3_[irrit_i] = accel_filter_2_[irrit_i] * filter_num[0] +
                             accel_filter_1_[irrit_i] * filter_num[1] +
                             accel_[irrit_i] * filter_num[2];
}

void Ins::apply_rotmatrix(const float input[3], float output[3])
{
  for (uint8_t i = 0; i < 3; i++) {
    output[i] =
      input[0] * rotMatrix_[i][0] + input[1] * rotMatrix_[i][1] + input[2] * rotMatrix_[i][2];
  }
}
// 旋转矩阵配置
void Ins::imu_cali_solve()
{
  apply_rotmatrix(bmi088_real_data_.gyro, gyro_);
  apply_rotmatrix(bmi088_real_data_.accel, accel_);
}

void Ins::setup_spi()
{
  hspi_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  if (HAL_SPI_Init(&hspi_) != HAL_OK) {
    Error_Handler();
  }

  // 初始化 DMA
  init_spi_dma(
    reinterpret_cast<uint32_t>(gyro_dma_tx_buf_), reinterpret_cast<uint32_t>(gyro_dma_rx_buf_),
    kSpiDmaGyroLength);
}

void Ins::init_spi_dma(uint32_t tx_buf, uint32_t rx_buf, uint16_t length)
{
  SET_BIT(hspi_.Instance->CR2, SPI_CR2_TXDMAEN);
  SET_BIT(hspi_.Instance->CR2, SPI_CR2_RXDMAEN);
  __HAL_SPI_ENABLE(&hspi_);

  init_dma_channel(hdma_spi_rx_, rx_buf, length, DMA_LISR_TCIF2);
  __HAL_DMA_ENABLE_IT(&hdma_spi_rx_, DMA_IT_TC);

  init_dma_channel(hdma_spi_tx_, tx_buf, length, DMA_LISR_TCIF3);
}

void Ins::init_dma_channel(
  DMA_HandleTypeDef & dma_handle, uint32_t buffer, uint16_t length, uint32_t clear_flag)
{
  __HAL_DMA_DISABLE(&dma_handle);

  while (dma_handle.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&dma_handle);
  }

  __HAL_DMA_CLEAR_FLAG(&dma_handle, clear_flag);

  dma_handle.Instance->PAR = (uint32_t) & (SPIx_->DR);
  dma_handle.Instance->M0AR = (uint32_t)(buffer);
  __HAL_DMA_SET_COUNTER(&dma_handle, length);
}
void Ins::gpioCallback(uint16_t GPIO_Pin, TaskHandle_t & INS_task_local_handler)
{
  if (GPIO_Pin == INT1_ACCEL_Pin) {
    accel_update_flag |= 1 << IMU_DR_SHFITS;
    accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag) {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == INT1_GYRO_Pin) {
    gyro_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag) {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == ExInterruptGpio_) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

void Ins::imu_cmd_spi_dma(void)
{
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if (
    (gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi_.hdmatx->Instance->CR & DMA_SxCR_EN) &&
    !(hspi_.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) &&
    !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
    gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
    gyro_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    spi_dma_enable(
      reinterpret_cast<uint32_t>(gyro_dma_tx_buf_), reinterpret_cast<uint32_t>(gyro_dma_rx_buf_),
      kSpiDmaGyroLength);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  if (
    (accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi_.hdmatx->Instance->CR & DMA_SxCR_EN) &&
    !(hspi_.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) &&
    !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
    accel_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    spi_dma_enable(
      reinterpret_cast<uint32_t>(accel_dma_tx_buf_), reinterpret_cast<uint32_t>(accel_dma_rx_buf_),
      kSpiDmaAccelLength);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  if (
    (accel_temp_update_flag & (1 << IMU_DR_SHFITS)) &&
    !(hspi_.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi_.hdmarx->Instance->CR & DMA_SxCR_EN) &&
    !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS))) {
    accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    spi_dma_enable(
      reinterpret_cast<uint32_t>(accel_temp_dma_tx_buf_),
      reinterpret_cast<uint32_t>(accel_temp_dma_rx_buf_), kSpiDmaAccelTempLength);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void Ins::spi_dma_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
  __HAL_DMA_DISABLE(&hdma_spi_rx_);
  __HAL_DMA_DISABLE(&hdma_spi_tx_);

  while (hdma_spi_rx_.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_spi_rx_);
  }
  while (hdma_spi_tx_.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_spi_tx_);
  }
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi_.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi_.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi_.hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi_.hdmarx));

  __HAL_DMA_CLEAR_FLAG(hspi_.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi_.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi_.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi_.hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi_.hdmatx));

  hdma_spi_rx_.Instance->M0AR = rx_buf;
  hdma_spi_tx_.Instance->M0AR = tx_buf;

  __HAL_DMA_SET_COUNTER(&hdma_spi_rx_, ndtr);
  __HAL_DMA_SET_COUNTER(&hdma_spi_tx_, ndtr);

  __HAL_DMA_ENABLE(&hdma_spi_rx_);
  __HAL_DMA_ENABLE(&hdma_spi_tx_);
}

void Ins::dmaIrqHandler(void)
{
  if (__HAL_DMA_GET_FLAG(hspi_.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_.hdmarx)) != RESET) {
    __HAL_DMA_CLEAR_FLAG(hspi_.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_.hdmarx));

    if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
      gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
      gyro_update_flag |= (1 << IMU_UPDATE_SHIFTS);
      HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    }

    if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
      accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_update_flag |= (1 << IMU_UPDATE_SHIFTS);
      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
      accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_temp_update_flag |= (1 << IMU_UPDATE_SHIFTS);
      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    imu_cmd_spi_dma();

    if (gyro_update_flag & (1 << IMU_UPDATE_SHIFTS)) {
      gyro_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
      gyro_update_flag |= (1 << IMU_NOTIFY_SHIFTS);
      __HAL_GPIO_EXTI_GENERATE_SWIT(ExInterruptGpio_);
    }
  }
}
}  // namespace io