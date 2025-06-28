# Vision Demo

新建applications/vision_task.cpp
```c++
#include "cmsis_os.h"
#include "io/vision/vision.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::Mahony imu;

sp::Vision vis(false, false);

extern "C" void vision_task()
{
  while (true) {
    // 推荐实际使用时放在imu_task
    vis.send(0, imu.q, imu.yaw, imu.vyaw, imu.pitch, imu.vpitch, 24.0f, 0);

    // 使用调试(f5)查看vis内部变量的变化

    osDelay(2); // 500Hz
  }
}

extern "C" void vision_callback(uint8_t * buf, uint32_t len) { vis.update(buf, len); }
```

修改Src/usbd_cdc_if.c中的`CDC_Receive_FS`函数
```c++
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  vision_callback(Buf, *Len);  // <- 添加这一行
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
```


编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/vision_task.cpp # <- 添加这一行
    sp_middleware/io/vision/vision.cpp # <- 添加这一行
    sp_middleware/tools/crc/crc.cpp # <- 添加这一行
    sp_middleware/tools/math_tools/math_tools.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```
