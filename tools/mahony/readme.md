# Mahony 姿态解算 Demo

### 编辑`CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/mahony/mahony.cpp # <- 添加这一行
    sp_middleware/tools/math_tools/math_tools.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

### 初始化

```cpp
#include "tools/mahony/mahony.hpp"

sp::Mahony imu(1e-3f);        // dt=1ms, kp=0.5(默认), ki=0.0(默认)
```

`dt`: 更新周期，单位为 s，需与任务调用周期严格一致

`kp`: 对重力方向的置信度。越高则越相信加速度计（pitch/roll 收敛更快），越低则越相信陀螺仪积分

`ki`: 积分增益，用于消除陀螺仪零偏，在校准过陀螺仪后通常置 0

### 调用

```cpp
imu.update(bmi088.acc, bmi088.gyro);

```

`acc`: 加速度计三轴读数，单位 m/s²

`gyro`: 陀螺仪三轴读数，单位 rad/s

每次调用后内部四元数、欧拉角及其微分均自动更新。

### 查询结果

```cpp
// 姿态四元数
float q[4] = imu.q;  // 只读！ {w, x, y, z}
                     // 含义：将同一个向量从载体系转换到地面系
                     // 即 v_world = q ⊗ v_body ⊗ q*

// 欧拉角（ZYX内旋顺序，载体系相对于地面系）
float yaw   = imu.yaw;    // 只读！ 单位：rad
float pitch = imu.pitch;  // 只读！ 单位：rad
float roll  = imu.roll;   // 只读！ 单位：rad

// 欧拉角速率（由陀螺仪角速度反解，数值上等于欧拉角的微分）
float vyaw   = imu.vyaw;    // 只读！ 单位：rad/s
float vpitch = imu.vpitch;  // 只读！ 单位：rad/s
float vroll  = imu.vroll;   // 只读！ 单位：rad/s

// 载体系下的角速度（陀螺仪原始值，未融合）
float wx = imu.w[0];  // 只读！ 单位：rad/s
float wy = imu.w[1];  // 只读！ 单位：rad/s
float wz = imu.w[2];  // 只读！ 单位：rad/s

// 几何 pitch 角（值域扩展到 ±π，可处理倒扣情况）
float pitch_geom  = imu.pitch_geom;   // 只读！ 单位：rad，范围 [-π, π]
float vpitch_geom = imu.vpitch_geom;  // 只读！ 单位：rad/s
```

### 说明

`vyaw` / `vpitch` / `vroll` 是欧拉角的微分，通过陀螺仪角速度反解得到，**并非直接对欧拉角做差分**,这样能减少微分噪声

只有当该 `Mahony` 对象解算的是**云台相对于底盘**的姿态或者底盘完全放在平地上时，`vyaw` / `vpitch` / `vroll` 才对应三个电机轴的角速度反馈值，可直接用于速度环。

`pitch_geom` 利用云台 x 轴和 z 轴在世界系下的投影判断象限，将 pitch 值域从 `[-π/2, π/2]` 扩展至 `[-π, π]`，在云台倒扣时仍然单调连续。可用于串腿翻倒后自启

### 动态修改增益

```cpp
imu.set_kp(0.5f);  // 动态修改 kp

```
用于解决打弹时候因为振动而影响加速度计读数,最终导致pitch值有误而发生的低头问题
只需要在打弹的时候将kp设置成0.01这种小量就行,在打弹结束再设置成0.5
