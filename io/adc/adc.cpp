#include "adc.hpp"

namespace sp
{
Adc::Adc(
  ADC_HandleTypeDef * adc, uint32_t channels, float vref, float voltage_divider,
  float voltage_offset)
: adc_(adc),
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

void Adc::update()
{
  //保险起见如果CUBE忘记配置Continue
  HAL_ADC_Start(adc_);
  this->voltage = HAL_ADC_GetValue(adc_) * this->adc12_to_v_coef_ + this->voltage_offset_;
}
}  // namespace sp
