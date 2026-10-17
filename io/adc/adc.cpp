#include "adc.hpp"

namespace sp
{
Adc::Adc(
  ADC_HandleTypeDef * adc, uint32_t channels, float vref, float voltage_divider,
  float voltage_offset)
: voltage(0.0f),
  adc_(adc),
  channel_pow_(1 << channels),
  vref_(vref),
  voltage_divider_(voltage_divider),
  voltage_offset_(voltage_offset)
{
}

void Adc::init()
{
  //C板 四分频（不超频又不功耗太高）
  adc_->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  HAL_ADC_Init(adc_);
  //计算ADC转换值到电压的系数
  this->adc12_to_v_coef_ = this->vref_ * this->voltage_divider_ / this->channel_pow_;
}

bool Adc::update()
{
  if (HAL_ADC_Start(adc_) != HAL_OK) return false;
  // 不能启动后立即读取 DR：必须等本次转换完成，避免上电零值或旧值误报警。
  if (HAL_ADC_PollForConversion(adc_, 1U) != HAL_OK) {
    HAL_ADC_Stop(adc_);
    return false;
  }
  const uint32_t raw = HAL_ADC_GetValue(adc_);
  if (HAL_ADC_Stop(adc_) != HAL_OK) return false;
  this->voltage = raw * this->adc12_to_v_coef_ + this->voltage_offset_;
  return true;
}
}  // namespace sp
