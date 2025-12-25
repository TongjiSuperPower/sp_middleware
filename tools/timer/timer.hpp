#ifndef SP__TIMER_HPP
#define SP__TIMER_HPP

#include <cstdint>

namespace sp
{

/**
 * @brief 微秒级计时器类（基于 DWT 硬件计数器）
 * @note  用于测量代码执行时间，精度可达 1/SystemCoreClock 秒（168MHz 下约 6ns）
 *
 * 使用示例：
 * @code
 *   sp::Timer timer;
 *   timer.start();
 *   // ... 待测代码 ...
 *   timer.stop();
 *   float dt_us = timer.delta_us();      // 获取耗时（微秒）
 *   float dt_ms = timer.delta_ms();      // 获取耗时（毫秒）
 * @endcode
 */
class Timer
{
public:
  /**
   * @brief 构造函数
   * @param auto_init 是否自动初始化 DWT（首次使用需要初始化）
   */
  Timer(bool auto_init = true);

  /**
   * @brief 初始化 DWT 计数器（全局只需调用一次）
   * @return true 初始化成功，false 失败（硬件不支持）
   */
  static bool init();

  /**
   * @brief 开始计时（记录起始时刻）
   */
  void start();

  /**
   * @brief 停止计时（记录结束时刻）
   */
  void stop();

  /**
   * @brief 获取时间间隔（微秒）
   * @return float 时间间隔，单位：us
   */
  float delta_us() const;

  /**
   * @brief 获取时间间隔（毫秒）
   * @return float 时间间隔，单位：ms
   */
  float delta_ms() const;

  /**
   * @brief 获取时间间隔（秒）
   * @return float 时间间隔，单位：s
   */
  float delta_s() const;

  /**
   * @brief 获取原始 tick 差值
   * @return uint32_t tick 数
   */
  uint32_t delta_tick() const;

  /**
   * @brief 获取当前时刻（微秒，从系统启动开始）
   * @return float 当前时刻，单位：us
   * @note  约 25 秒溢出一次（32位计数器 @ 168MHz）
   */
  static float now_us();

  // 统计功能
  float min_us() const { return min_delta_us_; }  ///< 最小耗时
  float max_us() const { return max_delta_us_; }  ///< 最大耗时
  uint32_t count() const { return count_; }       ///< 测量次数
  void reset_stats();                             ///< 重置统计

private:
  uint32_t start_tick_;  ///< 起始 tick
  uint32_t stop_tick_;   ///< 结束 tick
  float min_delta_us_;   ///< 最小耗时（微秒）
  float max_delta_us_;   ///< 最大耗时（微秒）
  uint32_t count_;       ///< 测量次数

  static bool initialized_;   ///< DWT 是否已初始化
  static float us_per_tick_;  ///< 每个 tick 对应的微秒数
};

}  // namespace sp

#endif  // SP__TIMER_HPP
