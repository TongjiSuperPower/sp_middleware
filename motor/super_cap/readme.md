# 使用方法

### 编辑`CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/motor/super_cap/super_cap.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```
### 初始化
```cpp
sp::SuperCap::SuperCap(SuperCapMode mode);
```
`mode`:超级电容的使用模式

使用`SuperCapMode`

### read
```cpp
supercap.read(can_1.rx_data, stamp_ms);
```

### write
```cpp
supercap.write(
    can_1.tx_data, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy,
    pm02.robot_status.power_management_chassis_output);
```

