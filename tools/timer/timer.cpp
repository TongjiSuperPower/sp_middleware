#include "timer.hpp"

// STM32 寄存器定义
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)    // DWT 控制寄存器
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)  // DWT 周期计数器
#define DEM_CR (*(volatile uint32_t *)0xE000EDFC)  // Debug Exception and Monitor Control Register
#define DEM_CR_TRCENA (1 << 24)                    // Trace enable bit
#define DWT_CTRL_CYCCNTENA (1 << 0)                // Cycle count enable bit

// STM32F407 系统时钟频率（根据实际配置修改）
#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK 168000000UL  // 168 MHz
#endif

namespace sp
{

// 静态成员初始化
bool Timer::initialized_ = false;
float Timer::us_per_tick_ = 1.0f / (SYSTEM_CORE_CLOCK / 1000000.0f);  // 168MHz -> ~0.00595 us/tick

Timer::Timer(bool auto_init)
: start_tick_(0), stop_tick_(0), min_delta_us_(1e9f), max_delta_us_(0.0f), count_(0)
{
  if (auto_init && !initialized_) {
    init();
  }
}

bool Timer::init()
{
  if (initialized_) {
    return true;
  }

  // 使能 DWT 外设
  DEM_CR |= DEM_CR_TRCENA;

  // 重置周期计数器
  DWT_CYCCNT = 0;

  // 使能周期计数器
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;

  // 验证是否使能成功
  if (DWT_CTRL & DWT_CTRL_CYCCNTENA) {
    initialized_ = true;
    return true;
  }

  return false;
}

void Timer::start() { start_tick_ = DWT_CYCCNT; }

void Timer::stop()
{
  stop_tick_ = DWT_CYCCNT;

  // 更新统计
  float dt = delta_us();
  if (dt < min_delta_us_) min_delta_us_ = dt;
  if (dt > max_delta_us_) max_delta_us_ = dt;
  count_++;
}

float Timer::delta_us() const
{
  // 处理溢出（32位计数器会回绕）
  uint32_t ticks = stop_tick_ - start_tick_;
  return static_cast<float>(ticks) * us_per_tick_;
}

float Timer::delta_ms() const { return delta_us() / 1000.0f; }

float Timer::delta_s() const { return delta_us() / 1000000.0f; }

uint32_t Timer::delta_tick() const { return stop_tick_ - start_tick_; }

float Timer::now_us() { return static_cast<float>(DWT_CYCCNT) * us_per_tick_; }

void Timer::reset_stats()
{
  min_delta_us_ = 1e9f;
  max_delta_us_ = 0.0f;
  count_ = 0;
}

}  // namespace sp
