# 使用方法

### 编辑`CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/gimbal/gimbal.cpp # <- 添加这一行
    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

### 初始化
```cpp    (单imu需要添加滤波器)
const sp::GimbalFilterConfig fc = {
  .yaw_angle = 0.1f,
  .pitch_angle = 0.1f,
  .roll_angle = 0.1f,
  .yaw_fdb_speed = 0.6f,
  .pitch_fdb_speed = 0.8f,
  .roll_fdb_speed = 0.6f,
  .yaw_target_speed = 0.6f,    // 与 yaw_fdb_speed 相同
  .pitch_target_speed = 0.8f,  // 与 pitch_fdb_speed 相同
  .yaw_target_acc = 0.03f,
  .pitch_target_acc = 0.03f,
};
sp::Gimbal gimbal(yaw_offset_ecd_angle, pitch_offset_ecd_angle, false, true, 1e-3f, fc);
```
`yaw_offset_ecd_angle`: 云台yaw轴码盘零点位置，单位为rad

`pitch_offset_ecd_angle`: 云台pitch轴码盘零点位置，单位为rad  

请确保该计算公式中`yaw_rel`和`pitch_rel`符合正方向定义（右手定则）
每个机器人的安装方式不同,不太清楚正负号就把所有四种可能都实验完,
最后保证:1.逆时针转电机是yaw_rel增加  2. 低头是pitch_rel增加


```cpp    (双imu理论上不需要添加滤波器)
sp::Gimbal gimbal();
```


### 调用

#### 更新云台状态 — 单IMU模式
```cpp
gimbal.update_all_single(gimbal_imu, yaw_angle, pitch_angle);
```
`gimbal_imu`: 云台IMU的Mahony姿态解算对象

`yaw_angle`: yaw轴编码器读数，单位为rad

`pitch_angle`: pitch轴编码器读数，单位为rad

通过云台IMU姿态与电机编码器角度，推算底盘在世界系下的姿态四元数，并计算云台相对底盘的欧拉角及角速度（含低通滤波）。同时完成重力补偿向量`k_torque`的更新。

#### 更新云台状态 — 双IMU模式
```cpp
gimbal.update_all_dual(gimbal_imu, chassis_imu);
```
`gimbal_imu`: 云台IMU的Mahony姿态解算对象

`chassis_imu`: 底盘IMU的Mahony姿态解算对象

直接由两个IMU的姿态四元数做差，解算云台相对底盘的等效欧拉角及角速度（含低通滤波）。理论上不需要编码器，精度更高。同时完成重力补偿向量`k_torque`的更新。

#### 计算目标值（自瞄 / 键鼠）
```cpp
gimbal.calc_all_target(gimbal_imu,
                       yaw_set_in_world, pitch_set_in_world,
                       vyaw_set_in_world, vpitch_set_in_world,
                       acc_yaw_set_in_world, acc_pitch_set_in_world);
```
`gimbal_imu`: 云台IMU的Mahony姿态解算对象

`yaw_set_in_world`: 世界系下目标yaw角，单位为rad

`pitch_set_in_world`: 世界系下目标pitch角，单位为rad

`vyaw_set_in_world`: 世界系下目标yaw角速度，单位为rad/s

`vpitch_set_in_world`: 世界系下目标pitch角速度，单位为rad/s

`acc_yaw_set_in_world`: 世界系下目标yaw角加速度，单位为rad/s²（键鼠模式可置0）

`acc_pitch_set_in_world`: 世界系下目标pitch角加速度，单位为rad/s²（键鼠模式可置0）

将世界系目标转换为云台相对底盘的目标角度、目标速度和目标加速度，单IMU与双IMU均使用同一函数。

### 查询结果

```cpp
// 由 update_all_single / update_all_dual 更新
float yaw_rel   = gimbal.yaw_rel;    //只读！ 云台yaw轴相对底盘的角度，单位：rad
float pitch_rel = gimbal.pitch_rel;  //只读！ 云台pitch轴相对底盘的角度，单位：rad

float yaw_spd   = gimbal.yaw_relative_speed;    //只读！ 云台yaw轴相对底盘的角速度，单位：rad/s
float pitch_spd = gimbal.pitch_relative_speed;  //只读！ 云台pitch轴相对底盘的角速度，单位：rad/s

// 由 calc_all_target 更新
float yaw_target_angle = gimbal.yaw_target_relative_angle;    //只读！ 目标yaw相对角度，单位：rad
float pitch_target_angle = gimbal.pitch_target_relative_angle;//只读！ 目标pitch相对角度，单位：rad

float yaw_target_spd = gimbal.yaw_target_relative_speed;    //只读！ 目标yaw相对角速度，单位：rad/s
float pitch_target_spd = gimbal.pitch_target_relative_speed;//只读！ 目标pitch相对角速度，单位：rad/s

float yaw_target_acc = gimbal.yaw_target_relative_acc;    //只读！ 目标yaw相对角加速度，单位：rad/s²
float pitch_target_acc = gimbal.pitch_target_relative_acc;//只读！ 目标pitch相对角加速度，单位：rad/s²

```
测试心得
1.对于键鼠的操控,应该使用并联pid,将角度环和速度环并联,这样能达到最好的效果
2.yaw_rel一定要和yaw_target_relative_angle 作比较作为pid的输入,因为我同时加了展开器,将会突破-pi,pi的值域
3.重力补偿系数cos(pitch)要用k_torque[2]代替
4.将原有的视觉发来的值和反馈值全部替换为gimbal类中的变量就行
---

### 旧接口

#### 更新云台状态(更新joint系下yaw和pitch的位置)
```cpp
gimbal.update(gimbal_imu, yaw_angle, pitch_angle);
```
`gimbal_imu`: Mahony姿态解算对象，包含云台IMU的roll、pitch、yaw角度

`yaw_angle`: yaw轴编码器读数，单位为rad

`pitch_angle`: pitch轴编码器读数，单位为rad

#### 计算关节空间设定值（根据世界系的目标值解算出joint系下的值）
```cpp
gimbal.calc(yaw_set_in_world, pitch_set_in_world);
```
`yaw_set_in_world`: 世界坐标系下的yaw轴目标角度，单位为rad

`pitch_set_in_world`: 世界坐标系下的pitch轴目标角度，单位为rad

### 查询结果

```cpp
//由update更新
float yaw_feedback = gimbal.yaw_fdb_in_joint;    //只读！ 云台yaw轴相对于joint系的反馈角度
float pitch_feedback = gimbal.pitch_fdb_in_joint;//只读！ 云台pitch轴相对于joint系的反馈角度

//由calc更新
float yaw_target = gimbal.yaw_set_in_joint;      //只读！ 云台yaw轴相对于joint系的设定角度
float pitch_target = gimbal.pitch_set_in_joint;  //只读！ 云台pitch轴相对于joint系的设定角度


```


# 原理与推导
见飞书云文档,搜索双稳