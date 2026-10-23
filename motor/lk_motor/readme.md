# LK电机

## 添加源文件

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/motor/lk_motor/lk_motor.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/
)
```

已经添加的源文件不需要重复添加。

## 初始化

```cpp
#include "motor/lk_motor/lk_motor.hpp"

sp::LK_Motor motor(1, sp::LK_Motors::MG4005, false);
```

- 第一个参数：电机ID，取值1~32。
- 第二个参数：电机型号，减速比由型号确定。
- 第三个参数：`true`使用多圈角度，`false`使用单圈角度。

| 型号 | 原装减速比 | 最大发送电流 |
|---|---|---|
| `sp::LK_Motors::MG4005` | 10:1 | ±8 A |
| `sp::LK_Motors::MHF9025` | 1:1 | ±15 A |
| `sp::LK_Motors::MF7025_28T` | 1:1 | ±15 A |

电流上限用于力矩指令限幅和速度指令的电流限制字段。
位置、增量位置指令的电流限制由驱动器设置决定。

上述最大电流作为峰值限制使用，相关系列说明书标注峰值持续时间10秒。
中间件只做幅值限制，不自动计时降额。

所以代码中必须要自己限制最大力矩！！！

## 参考例程：力矩控制

### 开启电机

```cpp
motor.write_enable(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

### 周期发送目标力矩

在控制任务中周期调用：

```cpp
motor.cmd_torque(0.5f);              // 输出端目标力矩0.5N·m
motor.write_torque(fdcan1.tx_data);  // 生成控制帧
fdcan1.send(motor.tx_id);            // 发送控制帧
```

### 接收反馈

接收端确认是标准数据帧且长度为8字节后，按ID交给电机解析：

```cpp
void lk_motor_recv(uint16_t rx_id, const uint8_t * data, uint32_t stamp_ms)
{
  if (rx_id == motor.rx_id) {
    motor.read(data, stamp_ms);
  }
}
```

`stamp_ms` 为当前时间，单位ms。

解析后可读取：

```cpp
motor.angle;             // 输出端角度，rad
motor.multicycle_angle;  // 输出端多圈角度，rad
motor.speed;             // 输出端速度，rad/s
motor.torque;            // 输出端估算力矩，N·m
motor.current;           // 转矩电流，A
motor.temperature;       // 电机温度，℃
```

多圈角度由上电后的编码器反馈累计得到，不是断电保存的绝对位置。

## 控制命令会自动返回状态2数据

力矩、速度和位置控制命令（0xA1～0xA8）执行后，电机会返回与状态2查询（0x9C）
相同布局的数据，只有命令字不同。

| 字节 | 回复内容 |
|---|---|
| `DATA[0]` | 对应的控制命令字 |
| `DATA[1]` | 电机温度 |
| `DATA[2..3]` | 转矩电流 |
| `DATA[4..5]` | 电机速度 |
| `DATA[6..7]` | 编码器位置 |

例如，发送 `0xA1` 力矩控制命令，回复的命令字也是 `0xA1`。

周期发送控制命令时，通常不需要额外发送 `write_state2()`。
收到的回复统一交给 `read()`，内部自动选择解析方式。

母线电压、母线电流、运行状态和错误位需要通过 `write_state1()` 单独查询。

## 其他控制模式

每种模式都先设置目标，再调用同名的组帧函数，最后发送。

### 速度控制

由电机内部速度环控制输出端转速：

```cpp
motor.cmd_velocity(2.0f);  // 输出端目标速度2rad/s
motor.write_velocity(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

### 单圈位置控制

指定电机轴单圈目标角度和转动方向：

```cpp
motor.cmd_position(1.5707963f, 0);  // 电机轴目标90°，顺时针
motor.write_position(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

角度单位rad，内部归一化到一圈。
方向：`0`为顺时针，`1`为逆时针。

此接口使用电机轴角度，不是减速箱后的输出端角度。

### 增量位置控制

让输出端按指定角度增量转动，正负号决定方向：

```cpp
motor.cmd_angle_increment(0.1f);  // 输出端增加0.1rad
motor.write_angle_increment(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

### 带限速的增量位置控制

指定输出端角度增量和最大速度：

```cpp
motor.cmd_angle_increment_speed(0.1f, 1.0f);  // 增加0.1rad，最大1rad/s
motor.write_angle_increment_speed(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

增量命令每发送一次都会下达一次增量请求，不要无条件周期重发。

## 状态查询

状态1包含温度、母线电压、母线电流、运行状态和错误标志：

```cpp
motor.write_state1(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

状态2包含温度、转矩电流、速度和编码器，可在未发送控制命令时主动查询：

```cpp
motor.write_state2(fdcan1.tx_data);
fdcan1.send(motor.tx_id);
```

所有回复统一交给 `motor.read()` 解析。

```cpp
motor.bus_voltage;  // 母线电压，V
motor.bus_current;  // 母线电流，A
motor.motor_state;  // 运行状态
motor.error_state;  // 错误标志
```

## 运行状态和错误标志

`motor_state`：

| 数值 | 含义 |
|---|---|
| `0x00` | 电机开启 |
| `0x10` | 电机关闭 |

`error_state` 各位为0表示正常，为1表示对应故障：

| 位 | 掩码 | 故障含义 |
|---|---|---|
| bit0 | `0x01` | 低压保护 |
| bit1 | `0x02` | 高压保护 |
| bit2 | `0x04` | 驱动过温 |
| bit3 | `0x08` | 电机过温 |
| bit4 | `0x10` | 电机过流 |
| bit5 | `0x20` | 电机短路 |
| bit6 | `0x40` | 电机堵转 |
| bit7 | `0x80` | 输入信号丢失超时 |

多个故障可以同时存在。例如，`0x05` 表示低压保护和驱动过温。

```cpp
if (motor.error_state & 0x40) {
  // 电机堵转
}
```

运行状态和错误标志通过状态1查询或清除错误命令的回复更新。
故障原因消除后，再发送 `write_clear_error()` 请求清除错误。

## 电机管理

以下函数生成指令后，都需要调用 `fdcan1.send(motor.tx_id)` 发送。

| 函数 | 作用 |
|---|---|
| `write_enable(data)` | 开启电机 |
| `write_disable(data)` | 关闭电机，清除驱动器圈数和控制指令 |
| `write_stop(data)` | 停止电机，保留开启状态 |
| `write_clear_error(data)` | 请求清除错误 |

## 在线判断

```cpp
motor.is_open();           // 是否收到过有效回复
motor.is_alive(stamp_ms);  // 最近100ms内是否收到有效回复
```

`is_open()` 不表示电机当前处于开启状态。
电机是否开启，应通过状态1回复中的 `motor_state` 判断。
