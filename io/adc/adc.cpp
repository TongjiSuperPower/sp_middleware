#include "adc.hpp"

namespace sp
{
adc::adc(ADC_HandleTypeDef * adc) : adc_(adc) {}

void adc::init()
{
  adc_->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  HAL_ADC_Init(&hadc3);
}

void adc::update()
{
  //保险起见如果CUBE忘记配置Continue
  HAL_ADC_Start(adc_);
  if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK) {
    this->buff_ = HAL_ADC_GetValue(adc_);
    this->voltage = this->buff_ * ADC12_TO_V_COEF;
  }
}

}  // namespace sp
