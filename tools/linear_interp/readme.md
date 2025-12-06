# Linear Interpolation Library

线性插值工具库，提供基本线性插值和查表插值功能。

## 功能特性

1. **基本线性插值函数** - 两点之间的简单插值
2. **查表线性插值类** - 基于预定义数据表的插值
3. **边界截断** - 超出范围时自动返回边界值
4. **高效查找** - 使用二分查找算法，时间复杂度 O(log n)

## 使用示例

### 1. 基本线性插值

```cpp
#include "tools/linear_interp/linear_interp.hpp"

// 在点 (0, 0) 和 (10, 100) 之间插值
float y = sp::linear_interp(5.0f, 0.0f, 0.0f, 10.0f, 100.0f);
// 结果: y = 50.0
```

### 2. 查表插值

```cpp
#include "tools/linear_interp/linear_interp.hpp"

// 定义查找表（x值必须从小到大排序）
const float x_table[] = {0.0f, 1.0f, 2.0f, 5.0f, 10.0f};
const float y_table[] = {0.0f, 10.0f, 15.0f, 30.0f, 100.0f};

// 创建插值对象
sp::LinearInterp interp(x_table, y_table, 5);

// 在范围内插值
float y1 = interp.calc(3.0f);  // 在 [2.0, 5.0] 区间内插值
// 结果: y1 ≈ 20.0

// 超出范围：返回边界值（自动截断）
float y2 = interp.calc(15.0f);  // 超出最大值 10.0
// 结果: y2 = 100.0 (返回最后一个y值)

float y3 = interp.calc(-1.0f);  // 低于最小值 0.0
// 结果: y3 = 0.0 (返回第一个y值)
```

### 3. 实际应用 - 电机力矩补偿

```cpp
// 速度-补偿力矩查找表
const float speed_table[] = {0.0f, 50.0f, 100.0f, 150.0f, 200.0f};
const float compensation_table[] = {0.5f, 0.8f, 1.2f, 1.8f, 2.5f};

sp::LinearInterp torque_comp(speed_table, compensation_table, 5);

void control_loop() {
  float current_speed = motor.speed;
  float compensation = torque_comp.calc(current_speed);
  
  float total_torque = pid.out + compensation;
  motor.cmd(total_torque);
}
```

### 4. 实际应用 - 非线性映射

```cpp
// 将摇杆输入 [-1, 1] 映射到非线性速度曲线
const float input_table[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
const float speed_table[] = {-2.0f, -0.3f, 0.0f, 0.3f, 2.0f};

sp::LinearInterp speed_curve(input_table, speed_table, 5);

void joystick_handler() {
  float joystick_input = remote.ch_lv;  // [-1, 1]
  float target_speed = speed_curve.calc(joystick_input);
  // 中间部分灵敏度降低，边缘部分加速
}
```

## API 参考

### 函数

```cpp
float linear_interp(float x, float x0, float y0, float x1, float y1);
```
- 基本两点线性插值
- 参数：
  - `x`: 要插值的点
  - `x0, y0`: 第一个点
  - `x1, y1`: 第二个点
- 返回：插值结果

### 类 LinearInterp

#### 构造函数
```cpp
LinearInterp(const float* x_table, const float* y_table, size_t size);
```
- `x_table`: x值数组指针（必须已排序，从小到大）
- `y_table`: y值数组指针
- `size`: 数组大小

#### 成员变量
- `float out` - 只读，最后一次计算结果

#### 成员函数
```cpp
float calc(float x);
```
- 计算插值
- 参数：要插值的x值
- 返回：插值结果（同时保存到 `out`）

## 注意事项

1. **x_table 必须已排序**：从小到大排列，否则结果不正确
2. **数据表生命周期**：LinearInterp 不拷贝数据，只保存指针，确保数据表在对象生命周期内有效
3. **线程安全**：单个对象不是线程安全的，多线程使用需要外部同步
4. **性能**：二分查找复杂度 O(log n)，适合频繁查询的场景

## 编译配置

在 `CMakeLists.txt` 中添加：
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/linear_interp/linear_interp.cpp
)
```

