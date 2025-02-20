# UI Demo

南航UI设计器（用于模拟UI绘制）：https://github.com/bismarckkk/RM-UI-Designer

![Image](https://github.com/user-attachments/assets/47958de8-eedc-4c69-95b9-b8032b03e88f)

新建applications/uart_task.cpp

```cpp
#include "cmsis_os.h"
#include "referee/pm02/pm02.hpp"
#include "referee/ui/ui.hpp"

// C板
sp::PM02 pm02(&huart6);

sp::UI_Manager ui_manager;

using namespace sp::ui;
Line line(Layer::LAYER_0, Color::RED_BLUE, 1, 500, 500, 600, 600);
Rectangle rect(Layer::LAYER_1, Color::YELLOW, 2, 1000, 550, 1100, 650);
Circle circle(Layer::LAYER_2, Color::GREEN, 3, 800, 600, 50);
Ellipse ellipse(Layer::LAYER_3, Color::ORANGE, 4, 700, 600, 50, 25);
Arc arc(Layer::LAYER_4, Color::PURPLE, 5, 600, 600, 0, 180, 25, 50);
Float float_num(Layer::LAYER_5, Color::PINK, 6, 500, 300, 60, 3.14);
Integer int_num(Layer::LAYER_6, Color::CYAN, 7, 500, 400, 70, 42);
String str(Layer::LAYER_7, Color::WHITE, 8, 500, 800, 80, "Hello, World!");

int int_num_value = 0;

extern "C" void uart_task()
{
  pm02.request();

  ui_manager.set_sender_id(sp::referee::robot_id::RED_HERO);  // 南航模拟器
  // ui_manager.set_sender_id(pm02.robot_status.robot_id);  // 官方裁判系统(建议放在循环里)

  ui_manager.delete_all();
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  ui_manager.pack(&str); // String类型每次只能发一个!
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  while (true) {
    ui_manager.pack(&line, &rect, &circle, &ellipse, &arc, &float_num, &int_num);
    pm02.send(ui_manager.data(), ui_manager.size());
    osDelay(33);

    // 南航模拟器显示更新好像不及时, 但是对应的数值在Properties面板中是在更新的
    int_num_value++;
    int_num.set_value(int_num_value);
    int_num.set_operate_type(OperateType::MODIFY);
    // 其余set方法请自行查看element.hpp头文件
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == pm02.huart) {
    pm02.request();
  }
}
```

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/uart_task.cpp # <- 添加这一行
    sp_middleware/referee/pm02/pm02.cpp # <- 添加这一行
    sp_middleware/referee/ui/element.cpp # <- 添加这一行
    sp_middleware/referee/ui/manager.cpp # <- 添加这一行
    sp_middleware/tools/crc/crc.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```