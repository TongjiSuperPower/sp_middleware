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

// Sampling period of loop(), in milliseconds. Transmit is interrupt driven, so
// this is the sample grid rather than the time a frame spends on the wire.
constexpr uint32_t PLOT_PERIOD_MS = 1;

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
//   sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos, sp::PlotUnit::Radian);
//
// The unit is only a label for the host's axis; it is not converted, and it is
// fixed by the first call that registers the topic.
//
// Registering is cheap (one slot of RAM), so paths can be left in the code
// permanently; only ticked topics occupy the serial link. See publish() for an
// O(1) handle when the same topic is written in a loop.
//
// Transmit is interrupt driven: flush() hands at most one frame to the UART
// and returns in microseconds, so calling it at 1 kHz does not cost the wire
// time of the frame. A frame that is still in flight makes the next flush skip
// its slot; tx_ is only refilled once the completion interrupt retires it.
//
// A task that only plots is written as two calls, with the record() lines in
// the loop hook:
//
//   super_plotter.initialize(scope_rows());
//   super_plotter.loop([]{ sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos); });
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
  // truncated into a different field. The optional unit is a label for the
  // host's axis (PlotUnit::Millimeter, PlotUnit::RadianPerSecond, ...), not a
  // conversion: the value is sent exactly as recorded.
  template <size_t N, typename T>
  static std::enable_if_t<std::is_arithmetic<T>::value, void> record(const char (&name)[N], T value)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    record_runtime(name, static_cast<float>(value));
  }

  template <size_t N, typename T>
  static std::enable_if_t<std::is_arithmetic<T>::value, void> record(
    const char (&name)[N], T value, PlotUnit unit)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    record_runtime(name, static_cast<float>(value), unit);
  }

  // Internal entry point used by tables that already checked their literals
  // (see scope_path()). There is no compile-time length check here, but
  // insert() rejects a name that does not fit, so it still never truncates.
  static void record_runtime(const char * name, float value, PlotUnit unit = PlotUnit::None);

  // Handle form: registers once, then set() is a single float store.
  template <size_t N>
  PlotPublisher publish(const char (&name)[N])
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    return publish_runtime(name);
  }

  template <size_t N>
  PlotPublisher publish(const char (&name)[N], PlotUnit unit)
  {
    static_assert(N <= PLOT_MAX_NAME, "SuperPlotter path is longer than PLOT_MAX_NAME");
    return publish_runtime(name, unit);
  }

  // Internal counterpart of publish(), same rules as record_runtime().
  PlotPublisher publish_runtime(const char * topic, PlotUnit unit = PlotUnit::None);

  // Starts reception on the UART. Call once from the plot task, before loop().
  void initialize();

  // Same, and also registers every row of a polled table: the table is
  // sampled on each tick from then on. N comes from the array itself, so the
  // caller never passes a count.
  //
  //   super_plotter.initialize(scope_rows());
  template <size_t N>
  void initialize(PolledRow (&rows)[N])
  {
    initialize();
    for (size_t i = 0; i < N; ++i) rows[i].pub = publish_runtime(rows[i].name, rows[i].unit);
    rows_ = rows;
    row_count_ = static_cast<int>(N);
  }

  // Per-tick user code. A captureless lambda converts, so the record() lines
  // can stay in the task next to the values they describe. Optional: a task
  // that only samples the polled table just calls loop().
  using TickFn = void (*)();

  // Runs forever on a PLOT_PERIOD_MS grid: samples the polled table, calls
  // record_hook, then hands at most one frame to the UART. Never returns; a
  // task that only plots is just initialize() + loop().
  void loop(TickFn record_hook = nullptr);

  void poll();
  void flush();
  void request();
  void on_rx(uint16_t size);

  // Called from HAL_UART_TxCpltCallback() once a frame has left the wire.
  // Marks the announcements in that frame as sent and retires the buffer.
  void on_tx_complete();

  // Frames the UART refused to take, or that were dropped because their
  // completion never arrived. Non-zero means the link, not the host parser,
  // is losing data.
  uint32_t tx_fail() const { return tx_fail_; }

  // Time the last frame spent on the wire, in microseconds. The CPU is free
  // during it, so this is bytes * 10 bits / baud rate, not the cost of
  // plotting; compare it with the frame period to see how much of the link is
  // in use.
  uint32_t last_tx_us() const { return last_tx_us_; }
  uint32_t max_tx_us() const { return max_tx_us_; }

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  TopicTable table_;
  // Polled table handed to initialize(), sampled by loop().
  PolledRow * rows_ = nullptr;
  int row_count_ = 0;
  uint8_t rx_buff_[128]{};
  uint8_t ring_[256]{};
  volatile uint16_t ring_w_ = 0;
  volatile uint16_t ring_r_ = 0;
  uint8_t tx_[PLOT_TX_SIZE]{};
  // One frame is in flight at a time: tx_ is handed to the UART and must not
  // be written again until the completion interrupt retires it.
  volatile bool tx_busy_ = false;
  // Announcement bytes in the frame that is in flight. Marked sent from
  // on_tx_complete(), so a frame that never made it out is retried.
  volatile size_t tx_announce_ = 0;
  uint32_t tx_start_cycles_ = 0;
  volatile uint32_t tx_fail_ = 0;
  volatile uint32_t last_tx_us_ = 0;
  volatile uint32_t max_tx_us_ = 0;

  void write(const char * name, float value, PlotUnit unit);
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
