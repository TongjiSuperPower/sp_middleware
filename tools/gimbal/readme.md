# 使用方法

### 编辑`CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/gimbal/gimbal.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```

### 初始化
```cpp
sp::Gimbal gimbal(yaw0, pitch0, reverse_yaw, reverse_yaw0, reverse_pitch, reverse_pitch0);
```
`yaw0`: 云台yaw轴码盘零点位置，单位为rad

`pitch0`: 云台pitch轴码盘零点位置，单位为rad  

请确保该计算公式中`yaw_realative_angle`和`pitch_realative_angle`符合正方向定义（右手定则）

``` 
  float yaw_realative_angle = sign_yaw_ * yaw_angle + sign_yaw0_ * yaw0_;
  float pitch_realative_angle = sign_pitch_ * pitch_angle + sign_pitch0_ * pitch0_;
```

### 调用

#### 更新云台状态
```cpp
gimbal.update(gimbal_imu, yaw_angle, pitch_angle);
```
`gimbal_imu`: Mahony姿态解算对象，包含云台IMU的roll、pitch、yaw角度

`yaw_angle`: yaw轴编码器读数，单位为rad

`pitch_angle`: pitch轴编码器读数，单位为rad

#### 计算关节空间设定值
```cpp
gimbal.calc(yaw_set_in_world, pitch_set_in_world);
```
`yaw_set_in_world`: 世界坐标系下的yaw轴目标角度，单位为rad

`pitch_set_in_world`: 世界坐标系下的pitch轴目标角度，单位为rad

### 查询结果

```cpp
float yaw_target = gimbal.yaw_set_in_joint;      // 云台yaw轴相对于码盘的设定角度
float pitch_target = gimbal.pitch_set_in_joint;  // 云台pitch轴相对于码盘的设定角度
float yaw_feedback = gimbal.yaw_fdb_in_joint;    // 云台yaw轴相对于码盘的反馈角度
float pitch_feedback = gimbal.pitch_fdb_in_joint;// 云台pitch轴相对于码盘的反馈角度
```

# 原理
见飞书云文档