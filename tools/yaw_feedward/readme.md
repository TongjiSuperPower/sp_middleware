# 使用方法

### 编辑`CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/yaw_feedward/yaw_feedward.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

### 初始化
```cpp
sp::YawFeedward yaw_feedward(dt,inertia,k_ff,damping,damping_out_max);
```
`dt`: 循环周期，单位为s

`inertia`: 转动惯量

`k_ff`: 前馈增益,默认值为1.0，需根据系统的实际动态响应，逐步调整

`damping`: 阻尼系数，默认值为0.0，需根据系统的实际动态响应，逐步调整

`damping_out_max`: 输出阻尼最大值，默认值为0.0，需根据系统的实际动态响应，逐步调整

### 调用

```cpp
yaw_feedward.calc(yaw_pos_pid.out); 
```

### 查询结果

```cpp
give_torque = yaw_feedward.out + yaw_speed_pid.out;
```

# 原理
电机的角加速度 α 和扭矩 T 之间的关系由旋转动力学方程描述：
$$T = J \cdot \alpha$$
其中：
- T：电机的输出扭矩（单位： $N·m$ ）
- J：电机的转动惯量（单位： $kg \cdot m^2$ ）
- α：角加速度（单位： $rad/s^2$ ）

前馈控制通常基于目标角速度 $\omega_{set}$ 和角加速度 $\alpha_{set}$ 来计算所需的补偿扭矩。

我们将位置环视为速度规划器

在离散系统中,位置环输出的差分即为目标加速度

$$\alpha_{set}[k] = \frac{\omega_{set}[k] - \omega_{set}[k-1]}{\Delta t}$$
