#ifndef SP__ICM42688_HPP
#define SP__ICM42688_HPP

#include "gpio.h"
#include "spi.h"

namespace sp
{
class ICM42688
{
public:
  // CS: SPI片选信号, 低电平时选中ICM42688
  // R_ab: icm42688原始坐标系{b}在机器人坐标系{a}下的坐标矩阵
  ICM42688(
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, uint16_t cs_pin, 
    const float r_ab[3][3], const float gyro_multipliers[3] = nullptr);

  float acc[3];   // 只读! 单位: m/s^2
  float gyro[3];  // 只读! 单位: rad/s
  float temp;     // 只读! 单位: celsius

  void init();
  void update();

private:
  SPI_HandleTypeDef * hspi_;
  GPIO_TypeDef * cs_port_;
  uint16_t cs_pin_;
  const float r_ab_[3][3];

  uint8_t rx_buff_[20];
  uint8_t tx_buff_[20];

  float gyro_multipliers_[3]; // 存储三轴标度因数

  uint8_t device_init();
  void read_registers(uint8_t reg, uint8_t len);
  void write_register(uint8_t reg, uint8_t data);
};

}  // namespace sp

#endif  // SP__ICM42688_HPP