#include "icm42688.hpp"

#include <cstring>
#include "icm42688_defs.h"
#include "cmsis_os.h"
#include "tools/math_tools/math_tools.hpp"

// 构造时配置
constexpr uint8_t ICM42688_ACC_RANGE_SET = ICM42688_ACC_RANGE_16G;
constexpr uint8_t ICM42688_GYRO_RANGE_SET = ICM42688_GYRO_RANGE_2000DPS;
constexpr uint8_t ICM42688_ODR_SET = ICM42688_ODR_1KHZ; // 设定1000Hz刷新率

// 根据量程计算缩放因子 (16G & 2000DPS)
constexpr float G_TO_MPS2 = 9.8f;
constexpr float ICM42688_ACC_INT_TO_G = 16.0f / 32768.0f;
constexpr float ICM42688_ACC_INT_TO_MPS2 = ICM42688_ACC_INT_TO_G * G_TO_MPS2;

constexpr float ICM42688_GYRO_INT_TO_DPS = 2000.0f / 32768.0f;
constexpr float ICM42688_GYRO_INT_TO_RPS = ICM42688_GYRO_INT_TO_DPS / 180.0f * sp::SP_PI;

// 温度转换系数 (Datasheet提供公式: Temp = val / 132.48 + 25)
constexpr float ICM42688_TEMP_FACTOR = 1.0f / 132.48f;
constexpr float ICM42688_TEMP_OFFSET = 25.0f;

constexpr size_t ICM42688_INIT_TABLE_SIZE = 3;

// clang-format off
const uint8_t ICM42688_INIT_TABLE[ICM42688_INIT_TABLE_SIZE][3] = {
  {ICM42688_PWR_MGMT0,     ICM42688_PWR_LN_MODE,                              ICM42688_PWR_MGMT_ERROR   },
  {ICM42688_ACCEL_CONFIG0, ICM42688_ACC_RANGE_SET | ICM42688_ODR_SET,         ICM42688_ACCEL_CONF_ERROR },
  {ICM42688_GYRO_CONFIG0,  ICM42688_GYRO_RANGE_SET | ICM42688_ODR_SET,        ICM42688_GYRO_CONF_ERROR  }
};
// clang-format on

namespace sp
{
ICM42688::ICM42688(
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, uint16_t cs_pin, 
  const float r_ab[3][3], const float gyro_multipliers[3])
: hspi_(hspi),
  cs_port_(cs_port),
  cs_pin_(cs_pin),
  r_ab_{
    {r_ab[0][0], r_ab[0][1], r_ab[0][2]},
    {r_ab[1][0], r_ab[1][1], r_ab[1][2]},
    {r_ab[2][0], r_ab[2][1], r_ab[2][2]}}
{
  if (gyro_multipliers != nullptr) {
    gyro_multipliers_[0] = gyro_multipliers[0];
    gyro_multipliers_[1] = gyro_multipliers[1];
    gyro_multipliers_[2] = gyro_multipliers[2];
  } else {
    gyro_multipliers_[0] = 1.0f;
    gyro_multipliers_[1] = 1.0f;
    gyro_multipliers_[2] = 1.0f;
  }
}

void ICM42688::init()
{
  while (device_init() != ICM42688_NO_ERROR) {
      osDelay(10); // 避免死循环卡死系统，加入适当延时等待
  }
}

uint8_t ICM42688::device_init()
{
  // 1. 软复位
  write_register(ICM42688_DEVICE_CONFIG, ICM42688_SOFT_RESET_VALUE);
  osDelay(10); // 等待复位完成

  // 2. 检查设备ID
  read_registers(ICM42688_WHO_AM_I, 1);
  osDelay(1);

  // rx_buff_[0]是虚拟字节，rx_buff_[1]是实际数据
  uint8_t whoami = rx_buff_[1];
  if (whoami != ICM42688_CHIP_ID_VALUE) return ICM42688_NO_SENSOR;

  // 3. 按照Table配置寄存器
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < ICM42688_INIT_TABLE_SIZE; i++) {
    reg = ICM42688_INIT_TABLE[i][0];

    write_register(reg, ICM42688_INIT_TABLE[i][1]);
    osDelay(2); // ICM42688 需要在配置电源后有短暂延时

    read_registers(reg, 1);
    osDelay(1);

    data = rx_buff_[1];
    if (data != ICM42688_INIT_TABLE[i][1]) return ICM42688_INIT_TABLE[i][2];
  }

  return ICM42688_NO_ERROR;
}

void ICM42688::update()
{
  // 突发读取从 0x1D (温度起始) 到 0x2A (陀螺仪Z轴结束)，共计 14 个字节
  read_registers(ICM42688_TEMP_DATA1, 14);

  // 第1个字节(rx_buff_[0]) 为寄存器地址占位，真实数据从rx_buff_[1]开始
  // ICM42688 传输是大端模式 (MSB First)
  int16_t temp_int   = (rx_buff_[1] << 8)  | rx_buff_[2];
  
  int16_t acc_x_int  = (rx_buff_[3] << 8)  | rx_buff_[4];
  int16_t acc_y_int  = (rx_buff_[5] << 8)  | rx_buff_[6];
  int16_t acc_z_int  = (rx_buff_[7] << 8)  | rx_buff_[8];

  int16_t gyro_x_int = (rx_buff_[9] << 8)  | rx_buff_[10];
  int16_t gyro_y_int = (rx_buff_[11] << 8) | rx_buff_[12];
  int16_t gyro_z_int = (rx_buff_[13] << 8) | rx_buff_[14];

  // 计算原始物理量
  float acc_x = acc_x_int * ICM42688_ACC_INT_TO_MPS2;
  float acc_y = acc_y_int * ICM42688_ACC_INT_TO_MPS2;
  float acc_z = acc_z_int * ICM42688_ACC_INT_TO_MPS2;

  float gyro_x = gyro_x_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[0];
  float gyro_y = gyro_y_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[1];
  float gyro_z = gyro_z_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[2];

  // 坐标系转换 (应用传入的旋转矩阵)
  this->acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
  this->acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
  this->acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;

  this->gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
  this->gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
  this->gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;

  this->temp = temp_int * ICM42688_TEMP_FACTOR + ICM42688_TEMP_OFFSET;
}

void ICM42688::read_registers(uint8_t reg, uint8_t len)
{
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

  tx_buff_[0] = reg | 0x80; // 最高位设为1代表读取

  // 读取总长度是 寄存器地址字节(1) + 数据长度(len)
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 1, 100);

  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

void ICM42688::write_register(uint8_t reg, uint8_t data)
{
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

  tx_buff_[0] = reg;       // 最高位设为0代表写入
  tx_buff_[1] = data;

  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

}  // namespace sp