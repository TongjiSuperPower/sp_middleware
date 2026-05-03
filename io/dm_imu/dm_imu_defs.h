#ifndef SP__DM_IMU_DEFS_H
#define SP__DM_IMU_DEFS_H

// ==================== DM-IMU-L1 寄存器地址 ====================

// --- 数据读取寄存器 (只读 R) ---
#define DM_IMU_REG_ACCEL 0x01  // 加速度数据
#define DM_IMU_REG_GYRO 0x02   // 角速度数据
#define DM_IMU_REG_EULER 0x03  // 欧拉角数据
#define DM_IMU_REG_QUAT 0x04   // 四元数数据

// --- 系统/控制寄存器 (只写 W) ---
#define DM_IMU_REG_RESTART 0x00        // 重启 IMU
#define DM_IMU_REG_ZERO_ANGLE 0x05     // 角度置零 (将当前姿态设为零点)
#define DM_IMU_REG_CALIB_ACC 0x06      // 启动加计六面校准
#define DM_IMU_REG_CALIB_GYRO 0x07     // 启动陀螺静态校准
#define DM_IMU_REG_CALIB_MAG 0x08      // 启动磁计椭球校准
#define DM_IMU_REG_SAVE_PARAM 0xFE     // 保存参数 (掉电不丢失)
#define DM_IMU_REG_FACTORY_RESET 0xFF  // 恢复出厂设置

// --- 通信/配置寄存器 (读写 RW) ---
#define DM_IMU_REG_COMM_MODE 0x09    // 切换通信模式
#define DM_IMU_REG_AUTO_RATE 0x0A    // 设置主动发送间隔
#define DM_IMU_REG_MODE_SWITCH 0x0B  // 切换主被动模式
#define DM_IMU_REG_BAUDRATE 0x0C     // 修改波特率
#define DM_IMU_REG_CAN_ID 0x0D       // CAN ID (接收ID)
#define DM_IMU_REG_MST_ID 0x0E       // MST ID (发送ID)
#define DM_IMU_REG_OUT_SELECT 0x0F   // 输出数据选择 (注:当前固件不支持修改)

// ==================== 数据解析线性映射极值 ====================
// (同之前，保持不变)
#define DM_IMU_ACCEL_MAX (235.2f)
#define DM_IMU_ACCEL_MIN (-235.2f)
#define DM_IMU_GYRO_MAX (34.88f)
#define DM_IMU_GYRO_MIN (-34.88f)
#define DM_IMU_PITCH_MAX (90.0f)
#define DM_IMU_PITCH_MIN (-90.0f)
#define DM_IMU_ROLL_MAX (180.0f)
#define DM_IMU_ROLL_MIN (-180.0f)
#define DM_IMU_YAW_MAX (180.0f)
#define DM_IMU_YAW_MIN (-180.0f)
#define DM_IMU_QUAT_MAX (1.0f)
#define DM_IMU_QUAT_MIN (-1.0f)

#endif  // SP__DM_IMU_DEFS_H