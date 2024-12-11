#include "low_pass_filter.hpp"

namespace sp
{
LowPassFilter::LowPassFilter(float alpha) : alpha_(alpha), inited_(false) {}

void LowPassFilter::update(float value)
{
  if (!inited_) {
    inited_ = true;
    last_ = value;
    this->out = value;
    return;
  }

  this->out = alpha_ * value + (1 - alpha_) * last_;
  last_ = this->out;
}

}  // namespace sp
