#ifndef SP__CYBERGEAR_MOTOR_HPP
#define SP__CYBERGEAR_MOTOR_HPP

#include <cstdint>
//气动cybergearID 0x06 0x00
#define Master_CAN_ID 0x00
#define CyberGear_CAN_ID 0x01
#define CYBERGEAR_MAX_TORQUE 12.0f            // N·m
#define CYBERGEAR_MAX_SPEED 30.0f             // 300 rpm = 31.42 rad/s
#define CYBERGEAR_MAX_POSITION 3.141593f * 4  // 2圈

//CyberGear支持的 communication_type 类型
#define Communication_Type_GetID 0x00          //反馈，获取设备ID：获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01  //控制，运动控制指令
#define Communication_Type_MotorRequest 0x02   //反馈，电机反馈数据
#define Communication_Type_MotorEnable 0x03    //控制，电机使能运行
#define Communication_Type_MotorStop 0x04      //控制，电机停止运行
#define Communication_Type_SetPosZero 0x06     //控制，设置电机机械零位
#define Communication_Type_CanID 0x07          //更改电机CAN ID
#define Communication_Type_Control_Mode 0x0C
#define Communication_Type_GetSingleParameter 0x11  //反馈，获取单个参数
#define Communication_Type_SetSingleParameter 0x12  //控制，设置单个参数
#define Communication_Type_ErrorFeedback 0x15       //反馈，故障反馈帧

//电机运行模式参数的index
#define run_mode 0x7005       //运行模式参数 0：运控模式，1：位置模式，2：速度模式，3：电流模式
#define iq_ref 0x7006         //电流模式Iq指令
#define spd_ref 0x700A        //转速模式转速指令
#define imit_torque 0x700B    //转矩限制
#define cur_kp 0x7010         //电流的Kp
#define cur_ki 0x7011         //电流的Ki
#define cur_filt_gain 0x7014  //电流滤波系数filter_gain
#define loc_ref 0x7016        //位置模式角度指令
#define limit_spd 0x7017      //位置模式速度设置
#define limit_cur 0x7018      //速度位置模式电流设置

namespace sp
{

class CyberGear_Motor
{
public:
  CyberGear_Motor(uint8_t master_id, uint8_t motor_id, float pmax, float vmax, float tmax);

  //主机id为master_id, 电机id为motor_id,电机接收控制帧的时候要指明来自哪个主机，电机反馈的时候要指明是哪台电机
  const uint8_t master_id;  //代表主机的ID
  const uint8_t motor_id;   //代表电机的ID

  //可以工作但状态异常，0：正常，1：欠压，2：过流，4：过温，8：磁编码器故障，16：HALL编码故障，32：未标定 多个错误相加
  uint8_t status_error = 0;
  //工作模式状态，0：Reset模式[复位]，1：Cali模式[标定]，2：Motor模式[运行]
  //初次使能电机会进入Reset模式，需要再次使能设定为Motor模式
  uint8_t mode = 0;

  //无法工作的故障帧反馈内容
  // bit16:A相电流采样过流
  // bit15~bit8:过载故障
  // bit7:编码器未标定
  // bit5:C相电流采样过流
  // bit4:B相电流采样过流
  // bit3:过压故障
  // bit2:欠压故障
  // bit1:驱动芯片故障
  // bit0:电机过温故障，默认80度
  uint32_t error = 0;
  // 只读! 警告温度，单位: 摄氏度*10
  uint32_t temp_warning = 0;

  //反馈的电机id
  uint8_t feedback_can_id;
  //反馈的主机id
  uint8_t feedback_master_id;
  //反馈的通信类型
  uint8_t feedback_communication_type;

  float angle = 0;        // 只读! 当前角度，[0~65535]对应(-4π，4π)
  float speed = 0;        // 只读! 当前角速度，[0~65535]对应(-30 rad/s，30 rad/s)
  float torque = 0;       // 只读! 当前力矩，[0~65535]对应(-12 N·m，12 N·m)
  float temperature = 0;  // 只读! 当前温度，Temp(摄氏度)*10

  uint16_t index = 0;      //参数索引
  float parameter = 0;     //参数值
  uint64_t MCU_ID = 0;     // 只读! MCU唯一标识符
  bool ID_gotten = false;  // 只读! 是否成功获取到设备ID

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  //读取电机反馈数据
  void read(uint32_t header, uint8_t * data, uint32_t stamp_ms);
  //把结构体的内容传到can发送数据中
  void write(uint8_t * data) const;
  //设置通信类型，并将各种数据传入Motor结构体
  //发送力矩
  void cmd(float torque);
  //获取设备ID
  void cmd_get_id();
  //设置电机CAN ID
  void cmd_set_can_id();
  //使能电机
  void cmd_motor_enable();
  //停止电机
  void cmd_motor_stop();
  //设置当前位置为零点
  void cmd_set_pos_zero();
  //读取单个参数
  void cmd_get_single_parameter(uint16_t index);
  //设置单个参数
  void cmd_set_single_parameter(uint16_t index, float parameter);

  //需要传递的参数
  uint16_t tar_torque = 0;
  uint8_t communication_type = 0;

private:
  const float pmax_;
  const float vmax_;
  const float tmax_;

  bool has_read_ = false;
  uint32_t last_read_ms_;

  //可以发给电机的变量,不需要使用
  uint16_t tar_position_ = 0;  //目标位置[0~65535]对应(-4π~4π)
  uint16_t tar_speed_ = 0;     //目标角速度[0~65535]对应(-30rad/s~30rad/s)
  uint16_t Kp_ = 0;            //Kp [0~65535]对应(0.0~500.0)
  uint16_t Kd_ = 0;            //Kd [0~65535]对应(0.0~5.0)

  uint16_t index_;      //参数索引
  uint32_t parameter_;  //参数值
};

}  // namespace sp
#endif