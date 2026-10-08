# ADC Demo

`sp::Adc` 负责转换结果到电压的换算。外设实例、通道和引脚由 CubeMX 配置，
构造函数里的第二个参数是 **ADC 位数**，不是通道号。

```cpp
#include "cmsis_os.h"
#include "io/adc/adc.hpp"

// C 板：PF10 / ADC3_IN8；
// sp::Adc battery(&hadc3, 12, 3.3f, (222.0f / 22.0f), 1.0f);
// 深大飞镖控制板专用配置如下
// F411：PC4 / ADC1_IN14；直接测量引脚电压，分压比 1，补偿 0V。
sp::Adc battery(&hadc1, 12, 3.3f, 1.0f, 0.0f);
float voltage = 0.0f;

extern "C" void adc_task(void * argument)
{
  (void)argument;
  battery.init();
  while (true) {
    if (battery.update()) {
      voltage = battery.voltage;  // 只使用本次成功转换的结果
    }
    osDelay(10);
  }
}
```
注：
1. todo：达妙
2. 可用作低电压报警，约23V左右为剩余一格，但电池间略有差异，且电机启动停止略有跳变，可以持续一段时间之后报警
3. 测量结果有0.2V左右跳变，且解算不加补偿时与真实值有一定误差，且C板之间也有一定个体差异
故可根据C板差异灵活调整补偿值voltage_offset

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/voltage_detect.cpp # <- 添加这一行
    sp_middleware/io/adc/adc.cpp # <- 添加这一行
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

C 板说明：

1. 约 23V 为电池剩余一格，但电池间有差异，电机启动/停止时也会跳变；
   可持续一段时间后再报警。
2. 原测量结果约有 0.2V 跳变，且 C 板之间存在个体差异，可调整
   `voltage_offset` 补偿；C 板补偿不可直接用于本工程。
3. 达妙板适配仍待实现。
