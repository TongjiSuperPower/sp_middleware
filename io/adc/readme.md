# ADC Demo

* CubeMX:
* PF10 使能ADC3_IN8

新建applications/voltage_detect.cpp
```cpp
#include "cmsis_os.h"
#include "io/adc/adc.hpp"

//C板
sp::Adc battery(
  &hadc3, 12, 3.3f, (222.0f / 22.0f), 1.0f);  // ADC3, 12位, 3.3V, 分压比222:22 ，偏移1V
float voltage = 0.0f;                           // 电压值
extern "C" void voltage_detect()
{
  battery.init();  // 初始化ADC
  while (1) {
    battery.update();
    voltage = battery.voltage;  // 获得电压值
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