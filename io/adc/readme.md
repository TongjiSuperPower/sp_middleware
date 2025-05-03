# Buzzer Demo
※※※
CubeMX：
PF10 使能ADC3_IN8

新建applications/voltage_detect.cpp
```cpp
#include "cmsis_os.h"
#include "io/adc/adc.hpp"

sp::adc battery(&hadc3);

//真实电压值
float voltage = 0.0f;
//电池电压补偿值
float voltage_offset = 2.5f;

extern "C" void voltage_detect()
{
  battery.init();
  while (1) {
    battery.update();
    voltage = battery.voltage + voltage_offset;
    osDelay(10);
  }
}
```
注：
1. todo：达妙
2. 测量结果有0.2V左右跳变，且解算不加补偿时与真实值有一定误差，且C板之间也有一定个体差异
故可根据C板差异灵活调整补偿值voltage_offset
//C板1
//真实值-测量值          误差值
//24.64——21.9           2.74
//22.87——20.3           2.57
//21.78——19.3           2.48
//19.79——17.6           2.19
//C板2
//真实值-测量值          误差值
//24.67——21.5           3.17
//22.83——19.9           2.93
//21.75——18.9           2.85
//19.80——17.3           2.50

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/adc_task.cpp # <- 添加这一行
    sp_middleware/io/adc/adc.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```