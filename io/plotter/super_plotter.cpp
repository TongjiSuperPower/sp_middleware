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
  if (huart_ == nullptr || huart_->gState != HAL_UART_STATE_READY) return;

  const size_t announced = table_.pop_announce(tx_, sizeof tx_);
  if (announced > 0) {
    send_bytes(tx_, static_cast<uint16_t>(announced));
    if (use_dma_ || huart_->gState != HAL_UART_STATE_READY) return;
  }
  const size_t sampled = table_.pop_sample(tx_, sizeof tx_);
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

void SuperPlotter::send_bytes(const uint8_t * data, uint16_t size)
{
  if (huart_ == nullptr || huart_->gState != HAL_UART_STATE_READY) return;
  if (use_dma_) {
    HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t *>(data), size);
  }
  else {
    HAL_UART_Transmit(huart_, const_cast<uint8_t *>(data), size, 5);
  }
}

}  // namespace sp
