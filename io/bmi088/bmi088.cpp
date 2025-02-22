#include "bmi088.hpp"

#include <cstring>

#include "bmi088_defs.h"
#include "cmsis_os.h"
#include "tools/math_tools/math_tools.hpp"

// TODO 构造时配置
constexpr uint8_t BMI088_ACC_RANGE_SET = BMI088_ACC_RANGE_6G;
constexpr uint8_t BMI088_GYRO_RANGE_SET = BMI088_GYRO_2000;

// ref: datasheet 5.3.4
constexpr float G_TO_MPS2 = 9.8f;
constexpr float BMI088_ACC_INT_TO_G = (1 << (BMI088_ACC_RANGE_SET + 1)) * 1.5f / 32768;
constexpr float BMI088_ACC_INT_TO_MPS2 = BMI088_ACC_INT_TO_G * G_TO_MPS2;

// ref: datasheet 5.5.2
constexpr float BMI088_GYRO_INT_TO_DPS = 2000.0f / (BMI088_GYRO_RANGE_SET + 1) / 32767;
constexpr float BMI088_GYRO_INT_TO_RPS = BMI088_GYRO_INT_TO_DPS / 180 * sp::PI;

// ref: datasheet 5.3.7
constexpr float BMI088_TEMP_FACTOR = 0.125f;
constexpr float BMI088_TEMP_OFFSET = 23.0f;

constexpr size_t BMI088_ACC_INIT_TABLE_SIZE = 6;
constexpr size_t BMI088_GYRO_INIT_TABLE_SIZE = 6;

// clang-format off

const uint8_t BMI088_ACC_INIT_TABLE[BMI088_ACC_INIT_TABLE_SIZE][3] = {
  {BMI088_ACC_PWR_CTRL      , BMI088_ACC_ENABLE_ACC_ON                                                       , BMI088_ACC_PWR_CTRL_ERROR      },
  {BMI088_ACC_PWR_CONF      , BMI088_ACC_PWR_ACTIVE_MODE                                                     , BMI088_ACC_PWR_CONF_ERROR      },
  {BMI088_ACC_CONF          , BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_800_HZ | BMI088_ACC_CONF_MUST_SET       , BMI088_ACC_CONF_ERROR          },
  {BMI088_ACC_RANGE         , BMI088_ACC_RANGE_SET                                                           , BMI088_ACC_RANGE_ERROR         },
  {BMI088_INT1_IO_CONF      , BMI088_ACC_INT1_OUT_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CONF_ERROR      },
  {BMI088_INT1_INT2_MAP_DATA, BMI088_DRDY_TO_INT1                                                            , BMI088_INT1_INT2_MAP_DATA_ERROR}
};

const uint8_t BMI088_GYRO_INIT_TABLE[BMI088_GYRO_INIT_TABLE_SIZE][3] = {
  {BMI088_GYRO_RANGE       , BMI088_GYRO_RANGE_SET                                   , BMI088_GYRO_RANGE_ERROR       },
  {BMI088_GYRO_BANDWIDTH   , BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET, BMI088_GYRO_BANDWIDTH_ERROR   },
  {BMI088_GYRO_LPM1        , BMI088_GYRO_NORMAL_MODE                                 , BMI088_GYRO_LPM1_ERROR        },
  {BMI088_GYRO_INT_CTRL    , BMI088_DRDY_ON                                          , BMI088_GYRO_CTRL_ERROR        },
  {BMI088_INT3_INT4_IO_CONF, BMI088_INT3_GPIO_PP | BMI088_INT3_GPIO_LOW              , BMI088_INT3_INT4_IO_CONF_ERROR},
  {BMI088_INT3_INT4_IO_MAP , BMI088_DRDY_IO_INT3                                     , BMI088_INT3_INT4_IO_MAP_ERROR }
};

// clang-format on

namespace sp
{
BMI088::BMI088(
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * csb1_port, uint16_t csb1_pin, GPIO_TypeDef * csb2_port,
  uint16_t csb2_pin, const float r_ab[3][3])
: hspi_(hspi),
  csb1_port_(csb1_port),
  csb2_port_(csb2_port),
  csb1_pin_(csb1_pin),
  csb2_pin_(csb2_pin),
  r_ab_{
    {r_ab[0][0], r_ab[0][1], r_ab[0][2]},
    {r_ab[1][0], r_ab[1][1], r_ab[1][2]},
    {r_ab[2][0], r_ab[2][1], r_ab[2][2]}}
{
}

void BMI088::init()
{
  while (acc_init() != BMI088_NO_ERROR);
  while (gyro_init() != BMI088_NO_ERROR);
}

void BMI088::update()
{
  acc_update();
  gyro_update();
}

// -------------------- 加速度计 --------------------

uint8_t BMI088::acc_init()
{
  // 上电后, 加速度计以I2C模式启动, 需要一次CSB1上升沿来切换到SPI模式, ref: datasheet 6.1
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 软复位
  acc_write(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  osDelay(1);

  // 软复位后, 加速度计以I2C模式启动, 需要一次CSB1上升沿来切换到SPI模式
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 读取ID
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 检查ID, 前两个字节为无效数据
  uint8_t whoami = rx_buff_[2];
  if (whoami != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

  // 配置寄存器
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < BMI088_ACC_INIT_TABLE_SIZE; i++) {
    reg = BMI088_ACC_INIT_TABLE[i][0];

    acc_write(reg, BMI088_ACC_INIT_TABLE[i][1]);
    osDelay(1);

    acc_read(reg, 1);
    osDelay(1);

    // 检查是否写入成功, 前两个字节为无效数据
    data = rx_buff_[2];
    if (data != BMI088_ACC_INIT_TABLE[i][1]) return BMI088_ACC_INIT_TABLE[i][2];
  }

  return BMI088_NO_ERROR;
}

void BMI088::acc_update()
{
  // 接收加速度计数据
  acc_read(BMI088_ACC_DATA, 6);

  // 前两个字节为无效数据
  int16_t acc_x_int = (rx_buff_[2 + 1] << 8) | rx_buff_[2 + 0];
  int16_t acc_y_int = (rx_buff_[2 + 3] << 8) | rx_buff_[2 + 2];
  int16_t acc_z_int = (rx_buff_[2 + 5] << 8) | rx_buff_[2 + 4];
  float acc_x = acc_x_int * BMI088_ACC_INT_TO_MPS2;
  float acc_y = acc_y_int * BMI088_ACC_INT_TO_MPS2;
  float acc_z = acc_z_int * BMI088_ACC_INT_TO_MPS2;

  // 前两个字节为无效数据
  acc_read(BMI088_TEMP_DATA, 2);
  int16_t temp_int = (rx_buff_[2 + 0] << 3) | (rx_buff_[2 + 1] >> 5);
  if (temp_int > 1023) temp_int -= 2048;  // ref: 5.3.7

  // 更新公开属性
  this->acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
  this->acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
  this->acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;
  this->temp = temp_int * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088::acc_read(uint8_t reg, uint8_t len)
{
  // 拉低CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.2
  tx_buff_[0] = reg | 0x80;

  // 前两个字节为无效数据
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 2, 100);

  // 拉高CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_SET);
}

void BMI088::acc_write(uint8_t reg, uint8_t data)
{
  // 拉低CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.2
  tx_buff_[0] = reg;
  tx_buff_[1] = data;

  // 只发不收
  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  // 拉高CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_SET);
}

// -------------------- 陀螺仪 --------------------

uint8_t BMI088::gyro_init()
{
  // 软复位
  gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  osDelay(1);

  // 读取ID
  gyro_read(BMI088_GYRO_CHIP_ID, 1);
  osDelay(1);

  // 检查ID, 第一个字节为无效数据
  uint8_t whoami = rx_buff_[1];
  if (whoami != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

  // 配置寄存器
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < BMI088_GYRO_INIT_TABLE_SIZE; i++) {
    reg = BMI088_GYRO_INIT_TABLE[i][0];

    gyro_write(reg, BMI088_GYRO_INIT_TABLE[i][1]);
    osDelay(1);

    gyro_read(reg, 1);
    osDelay(1);

    // 检查是否写入成功, 第一个字节为无效数据
    data = rx_buff_[1];
    if (data != BMI088_GYRO_INIT_TABLE[i][1]) return BMI088_GYRO_INIT_TABLE[i][2];
  }

  return BMI088_NO_ERROR;
}

void BMI088::gyro_update()
{
  // 接收陀螺仪数据
  gyro_read(BMI088_GYRO_DATA, 6);

  // 第一个字节为无效数据
  int16_t gyro_x_int = (rx_buff_[1 + 1] << 8) | rx_buff_[1 + 0];
  int16_t gyro_y_int = (rx_buff_[1 + 3] << 8) | rx_buff_[1 + 2];
  int16_t gyro_z_int = (rx_buff_[1 + 5] << 8) | rx_buff_[1 + 4];
  float gyro_x = gyro_x_int * BMI088_GYRO_INT_TO_RPS;
  float gyro_y = gyro_y_int * BMI088_GYRO_INT_TO_RPS;
  float gyro_z = gyro_z_int * BMI088_GYRO_INT_TO_RPS;

  // 更新公开属性
  this->gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
  this->gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
  this->gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;
}

void BMI088::gyro_read(uint8_t reg, uint8_t len)
{
  // 拉低CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.1
  tx_buff_[0] = reg | 0x80;

  // 第一个字节为无效数据
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 1, 100);

  // 拉高CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_SET);
}

void BMI088::gyro_write(uint8_t reg, uint8_t data)
{
  // 拉低CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.1
  tx_buff_[0] = reg;
  tx_buff_[1] = data;

  // 只发不收
  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  // 拉高CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_SET);
}

}  // namespace sp
