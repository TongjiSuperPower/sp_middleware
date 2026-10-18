#ifndef SP__SUPER_PLOTTER_HPP
#define SP__SUPER_PLOTTER_HPP

#include <type_traits>

#include "plot_link.hpp"
#include "usart.h"

namespace sp
{
// Bytes of the frame buffer. A sample frame is 3 + 5 * k, so this sets how many
// values travel in one frame; a larger subscribed set is spread over several
// flushes rather than dropped.
constexpr int PLOT_TX_SIZE = 192;
static_assert(PLOT_TX_SIZE >= 3 + 5 * 4, "tx_ must carry at least four values");
static_assert(PLOT_MAX_TOPICS <= PLOT_MAX_ID, "topic id is one byte on the wire");

// Named-topic plotter with host-controlled subscriptions.
//
// Unlike Plotter (fixed AA BB | size | float[18] frames, no names), the host
// learns every topic name from AA BC announcements and then asks for only the
// topics the user ticked (AA BD). A frame therefore costs 5 bytes per topic
// instead of always sending 18 floats.
//
// The usual way to publish is the static record(), which can be called from
// any task at any time -- the topic is registered on the first call:
//
//   sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos);
//
// Registering is cheap (one slot of RAM), so paths can be left in the code
// permanently; only ticked topics occupy the serial link. See publish() for an
// O(1) handle when the same topic is written in a loop.
class SuperPlotter
{
public:
  SuperPlotter(UART_HandleTypeDef * huart, bool use_dma = true);

  UART_HandleTypeDef * huart() const { return huart_; }

  // Records a value, registering the topic the first time the name is seen.
  // Forwards to the instance created in plot_task.cpp, so nothing needs to be
  // passed around: include this header and call it.
  //
  // Paths must be literals so the length is known at compile time; a path that
  // does not fit in PLOT_MAX_NAME is a compile error rather than being
  // truncated into a different field.
  template <size_t N, typename T>
  static std::enable_if_t<std::is_arithmetic<T>::value, void> record(const char (&name)[N], T value)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    record_runtime(name, static_cast<float>(value));
  }

  // Internal entry point used by tables that already checked their literals
  // (see scope_path()). There is no compile-time length check here, but
  // insert() rejects a name that does not fit, so it still never truncates.
  static void record_runtime(const char * name, float value);

  // Handle form: registers once, then set() is a single float store.
  template <size_t N>
  PlotPublisher publish(const char (&name)[N])
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    return publish_runtime(name);
  }

  // Internal counterpart of publish(), same rules as record_runtime().
  PlotPublisher publish_runtime(const char * topic);

  void poll();
  void flush();
  void request();
  void on_rx(uint16_t size);

  // Frames the UART refused to take. Non-zero means the link, not the host
  // parser, is losing data.
  uint32_t tx_fail() const { return tx_fail_; }

  // Time the last blocking transmit occupied the CPU, in microseconds. Split
  // from the frame period this separates "the wire is slow" from "the task is
  // not being scheduled".
  uint32_t last_tx_us() const { return last_tx_us_; }
  uint32_t max_tx_us() const { return max_tx_us_; }

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  TopicTable table_;
  uint8_t rx_buff_[128]{};
  uint8_t ring_[256]{};
  volatile uint16_t ring_w_ = 0;
  volatile uint16_t ring_r_ = 0;
  uint8_t tx_[PLOT_TX_SIZE]{};
  volatile uint32_t tx_fail_ = 0;
  uint32_t last_tx_us_ = 0;
  uint32_t max_tx_us_ = 0;

  void write(const char * name, float value);
  bool send_bytes(const uint8_t * data, uint16_t size);
};

// Shorthand for SuperPlotter::record(). Same literal-only rule: a path that
// does not fit in PLOT_MAX_NAME fails to compile.
template <size_t N, typename T>
inline std::enable_if_t<std::is_arithmetic<T>::value, void> record(const char (&name)[N], T value)
{
  static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
  SuperPlotter::record_runtime(name, static_cast<float>(value));
}

}  // namespace sp

#endif  // SP__SUPER_PLOTTER_HPP
