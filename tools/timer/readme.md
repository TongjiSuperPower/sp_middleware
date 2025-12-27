# Timer 微秒级计时器

基于 ARM Cortex-M4 DWT（Data Watchpoint and Trace）硬件计数器的高精度计时工具。

## 特性

- **高精度**：168MHz 主频下精度约 6ns，远超微秒级需求
- **零开销**：直接读取硬件寄存器，无系统调用
- **统计功能**：自动记录最小/最大耗时和测量次数
- **溢出处理**：正确处理 32 位计数器回绕

## 使用方法

### 编辑 `CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/timer/timer.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

### 初始化

```cpp
#include "tools/timer/timer.hpp"

sp::Timer timer;  // 自动初始化 DWT
```

`auto_init`: 是否自动初始化 DWT（默认 true，首次使用需要初始化）

### 基本使用

```cpp
timer.start();

// ... 待测代码 ...
mahony.update(gyro, accel, dt);
gimbal.update_q(imu, yaw_angle, pitch_angle);

timer.stop();

// 获取耗时
float dt_us = timer.delta_us();   // 微秒
float dt_ms = timer.delta_ms();   // 毫秒
float dt_s  = timer.delta_s();    // 秒
```

### 查看统计信息

```cpp
float min = timer.min_us();    // 最小耗时（微秒）
float max = timer.max_us();    // 最大耗时（微秒）
uint32_t n = timer.count();    // 测量次数

timer.reset_stats();           // 重置统计
```

### 多段测量

```cpp
sp::Timer timer_imu;
sp::Timer timer_gimbal;
sp::Timer timer_total;

void control_loop()
{
  timer_total.start();

  timer_imu.start();
  mahony.update(...);
  timer_imu.stop();

  timer_gimbal.start();
  gimbal.update_q(...);
  timer_gimbal.stop();

  timer_total.stop();

  // 分别查看各段耗时
  float t_imu = timer_imu.delta_us();
  float t_gimbal = timer_gimbal.delta_us();
  float t_total = timer_total.delta_us();
}
```

### 获取当前时刻

```cpp
float now = sp::Timer::now_us();  // 从系统启动开始的微秒数
```

**注意**：约 25 秒溢出一次（32位计数器 @ 168MHz）

## API 参考

| 方法                           | 说明                       |
| ------------------------------ | -------------------------- |
| `Timer(bool auto_init = true)` | 构造函数                   |
| `static bool init()`           | 初始化 DWT（全局只需一次） |
| `void start()`                 | 开始计时                   |
| `void stop()`                  | 停止计时                   |
| `float delta_us()`             | 获取时间间隔（微秒）       |
| `float delta_ms()`             | 获取时间间隔（毫秒）       |
| `float delta_s()`              | 获取时间间隔（秒）         |
| `uint32_t delta_tick()`        | 获取原始 tick 差值         |
| `static float now_us()`        | 获取当前时刻（微秒）       |
| `float min_us()`               | 最小耗时统计               |
| `float max_us()`               | 最大耗时统计               |
| `uint32_t count()`             | 测量次数统计               |
| `void reset_stats()`           | 重置统计数据               |

## 原理

DWT 是 ARM Cortex-M3/M4 内核自带的调试组件，其中 `DWT_CYCCNT` 寄存器是一个 32 位周期计数器，每个 CPU 时钟周期递增一次。

- **地址**：`0xE0001004`
- **频率**：与 CPU 主频相同（168MHz）
- **精度**：1 / 168MHz ≈ 5.95ns
- **溢出周期**：2^32 / 168MHz ≈ 25.6 秒
