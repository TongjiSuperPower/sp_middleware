#ifndef IO__BMI088_HPP
#define IO__BMI088_HPP

#include "gpio.h"
#include "spi.h"

namespace io
{
class BMI088
{
public:
  // CSB1: SPI片选信号, 低电平时选中加速度计
  // CSB2: SPI片选信号, 低电平时选中陀螺仪
  BMI088(
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * csb1_port, uint16_t csb1_pin, GPIO_TypeDef * csb2_port,
    uint16_t csb2_pin);

  float acc[3];       // 只读! 单位: m/s^2
  float gyro[3];      // 只读! 单位: rad/s
  float temperature;  // 只读! 单位: celsius

  void init();
  void update();

private:
  SPI_HandleTypeDef * hspi_;
  GPIO_TypeDef * csb1_port_;
  GPIO_TypeDef * csb2_port_;
  uint16_t csb1_pin_;
  uint16_t csb2_pin_;

  uint8_t rx_buff_[8];
  uint8_t tx_buff_[8];

  uint8_t acc_init();
  void acc_update();
  void acc_read(uint8_t reg, uint8_t len);
  void acc_write(uint8_t reg, uint8_t data);

  uint8_t gyro_init();
  void gyro_update();
  void gyro_read(uint8_t reg, uint8_t len);
  void gyro_write(uint8_t reg, uint8_t data);
};

}  // namespace io

#endif  // IO__BMI088_HPP