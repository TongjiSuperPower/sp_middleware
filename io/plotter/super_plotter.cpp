#include "super_plotter.hpp"

namespace sp
{
namespace
{
// Set by the SuperPlotter constructor. Global because record() is a static
// entry point that has no instance to work with.
SuperPlotter * g_instance = nullptr;

// Registration happens at most kMaxTopics times over the life of the program,
// so masking interrupts here is cheap. PRIMASK is saved and restored so this
// nests correctly inside another critical section.
uint32_t plot_lock()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void plot_unlock(uint32_t primask) { __set_PRIMASK(primask); }
}  // namespace

SuperPlotter::SuperPlotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma)
{
  table_.set_lock(&plot_lock, &plot_unlock);
  g_instance = this;
  // Cycle counter used to time a transmit. Only diagnostics: it separates a
  // slow UART from a starved task when the frame rate is wrong.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void SuperPlotter::record_runtime(const char * name, float value)
{
  if (g_instance == nullptr) return;
  g_instance->write(name, value);
}

void SuperPlotter::write(const char * name, float value) { table_.record(name, value); }

PlotPublisher SuperPlotter::publish_runtime(const char * topic) { return table_.publish(topic); }

void SuperPlotter::poll()
{
  while (ring_r_ != ring_w_) {
    const uint8_t byte = ring_[ring_r_ & 0xff];
    ring_r_ = static_cast<uint16_t>(ring_r_ + 1);
    table_.ingest(&byte, 1);
  }
}

void SuperPlotter::flush()
{
  poll();
  if (huart_ == nullptr) return;

  // One frame at a time: tx_ has been handed to the UART and cannot be filled
  // again until the completion interrupt retires it.
  if (tx_busy_) {
    // The HAL sets gState back to READY before it calls the completion
    // callback, and a task cannot run in between because the callback is in
    // the same interrupt. So seeing it ready here means the transfer ended on
    // an error path that never reported completion. A 50 byte frame needs
    // 0.6 ms; still busy after 10 ms means the interrupt was lost. Either way
    // drop the frame and let the next flush retry, rather than stop plotting
    // for the rest of the run.
    if (huart_->gState == HAL_UART_STATE_READY) {
      tx_busy_ = false;
      tx_announce_ = 0;
      ++tx_fail_;
    }
    else if (DWT->CYCCNT - tx_start_cycles_ > SystemCoreClock / 100u) {
      HAL_UART_AbortTransmit(huart_);
      tx_busy_ = false;
      tx_announce_ = 0;
      ++tx_fail_;
    }
    return;
  }
  if (huart_->gState != HAL_UART_STATE_READY) return;

  // Names are sent one per flush. A single ~180-byte name dump was truncated
  // on this link; one name is at most 19 bytes.
  constexpr size_t kAnnounceBurst = 3 + 15 + 1;
  // 8 values is 44 bytes, 0.48 ms on the wire, so at a 1 ms period the link
  // is half used and there is room for the adapter between frames. 16 values
  // would use 91% of the wire and overrun it.
  constexpr size_t kSampleBurst = 3 + 5 * 8 + 1;

  const size_t announced = table_.pop_announce(tx_, kAnnounceBurst);
  if (announced > 0) {
    // Stored before the transfer starts so the completion interrupt, which
    // may run before send_bytes() returns, always sees it.
    tx_announce_ = announced;
    if (!send_bytes(tx_, static_cast<uint16_t>(announced))) tx_announce_ = 0;
    return;
  }
  const size_t sampled = table_.pop_sample(tx_, kSampleBurst);
  if (sampled > 0) send_bytes(tx_, static_cast<uint16_t>(sampled));
}

void SuperPlotter::request()
{
  if (huart_ == nullptr) return;
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buff_, sizeof rx_buff_);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(huart_, rx_buff_, sizeof rx_buff_);
  }
}

void SuperPlotter::on_rx(uint16_t size)
{
  if (size > sizeof rx_buff_) size = sizeof rx_buff_;
  for (uint16_t i = 0; i < size; ++i) {
    ring_[ring_w_ & 0xff] = rx_buff_[i];
    ring_w_ = static_cast<uint16_t>(ring_w_ + 1);
  }
}

// Queues a frame and returns: the bytes leave in the UART interrupt, so the
// 1 kHz plot loop costs microseconds instead of the ~0.5 ms a 44 byte frame
// needs on the wire. tx_ stays valid until on_tx_complete().
bool SuperPlotter::send_bytes(const uint8_t * data, uint16_t size)
{
  if (huart_ == nullptr || data == nullptr || size == 0) return false;
  // Set before the call: the first interrupt can fire inside it.
  tx_busy_ = true;
  tx_start_cycles_ = DWT->CYCCNT;
  const HAL_StatusTypeDef status =
    use_dma_ ? HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t *>(data), size)
             : HAL_UART_Transmit_IT(huart_, const_cast<uint8_t *>(data), size);
  if (status != HAL_OK) {
    tx_busy_ = false;
    ++tx_fail_;
    return false;
  }
  return true;
}

void SuperPlotter::on_tx_complete()
{
  if (huart_ == nullptr) return;
  if (tx_busy_) {
    // The CPU was free for the whole transfer, so this measures the wire time
    // (bytes * 10 bits / baud), not the cost of plotting.
    const uint32_t cycles = DWT->CYCCNT - tx_start_cycles_;
    last_tx_us_ = cycles / (SystemCoreClock / 1000000u);
    if (last_tx_us_ > max_tx_us_) max_tx_us_ = last_tx_us_;
    // Announcements are only marked sent now that the bytes are actually out:
    // a frame that is aborted or half sent is retried instead of leaving the
    // host with a torn name.
    if (tx_announce_ != 0) {
      table_.commit_announce(tx_announce_);
      tx_announce_ = 0;
    }
    tx_busy_ = false;
  }
}

}  // namespace sp
