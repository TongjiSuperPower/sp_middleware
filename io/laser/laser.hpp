#ifndef SP__LASER_GY56_HPP
#define SP__LASER_GY56_HPP

#include <cstdint>

#include "usart.h"

namespace sp
{
enum class LaserMode : uint8_t
{
  UNKNOWN = 0,  // 尚未从有效数据帧获取模式
  SHORT = 1,    // 短距离模式
  MEDIUM = 2,   // 中距离模式
  LONG = 3      // 长距离模式
};

class LaserGY56
{
public:
  explicit LaserGY56(UART_HandleTypeDef * huart);

  // 返回最近一帧有效距离, 单位: cm; 使用前通过is_valid()判断是否过期
  uint16_t get_distance() const;
  // 收到过有效帧且距上一帧不超过timeout_ms时返回true, 单位: ms
  bool is_valid(uint32_t timeout_ms = 500) const;
  // 返回最近一帧有效数据中报告的模式
  LaserMode get_mode() const;

  // 返回命令是否通过串口发送成功
  bool send_mode(LaserMode mode);
  bool set_baud_rate_9600();
  bool set_baud_rate_115200();
  bool save_config();

  // 启动USART的DMA空闲接收; 回调传入本次收到的字节数
  bool start_receive();
  void receive_complete(uint16_t size);
  void update(uint8_t byte);

private:
  static constexpr uint8_t FRAME_SIZE = 8;
  static constexpr uint16_t RX_BUFFER_SIZE = 64;

  bool send_command(uint8_t command);
  bool frame_valid() const;
  void parse_frame();

  UART_HandleTypeDef * huart_;
  uint8_t rx_buff_[RX_BUFFER_SIZE];
  uint8_t frame_[FRAME_SIZE];
  uint8_t frame_size_;

  volatile uint16_t distance_cm_;
  volatile uint32_t last_frame_tick_;
  volatile LaserMode mode_;
  volatile bool has_reliable_distance_;
};

}  // namespace sp

#endif  // SP__LASER_GY56_HPP
