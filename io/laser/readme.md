
新建applications/laser_task.cpp
```cpp
#include "laser_task.hpp"

#include "cmsis_os.h"

sp::LaserGY56 laser(&huart1);
uint16_t distance = 0xFFFF;

extern "C" void laser_task()
{
  laser.start_receive();
  laser.set_baud_rate_115200();
  laser.save_config();

  while (true) {
    // 对外只发布未超时的有效距离, 否则发布无效值
    distance = laser.is_valid() ? laser.get_distance() : 0xFFFF;
    osDelay(1);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart1) {
    laser.receive_complete(Size);
  }

}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart1) {
    laser.start_receive();
  }
}

```
`send_mode()`、`set_baud_rate_9600()`、`set_baud_rate_115200()` 和 `save_config()` 均发送 3 字节命令，返回 HAL 发送是否成功。

## Note

- `get_distance()` 返回最近一帧有效测距的厘米值（cm），本身不检查超时。`is_valid()` 默认使用 500 ms 有效期；任务导出的 `distance` 也是 cm，在无有效帧或超时后为 `0xFFFF`。
- 接收解析保留 8 字节帧头、数据类型、校验和及状态检查。只有状态为 0 的有效帧才更新距离和模式；DMA 分段或帧头错位时仍使用滑动窗口重新组帧。
- 设置命令仍为 3 字节格式：9600 为 `A5 AE 53`，115200 为 `A5 AF 54`，保存配置为 `A5 25 CA`。校验字节是前两个字节之和的低 8 位。
- 模块出厂默认 9600，需要先按 9600 完成切换，再使用当前 115200 固件。
- USART1 由 Laser 使用。`plotter_task` 中的 Plotter 发送目前被注释，不应与 Laser 同时向 `huart1` 发数据。

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/laser_task.cpp # <- 添加这一行
    sp_middleware/io/laser/laser.cpp # <- 添加这一行

)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```
