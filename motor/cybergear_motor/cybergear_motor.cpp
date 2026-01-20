#include "cybergear_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
CyberGear_Motor::CyberGear_Motor(
  uint8_t master_id, uint8_t motor_id, float pmax, float vmax, float tmax)
: master_id(master_id), motor_id(motor_id), pmax_(pmax), vmax_(vmax), tmax_(tmax)
{
}

bool CyberGear_Motor::is_open() const { return has_read_; }

bool CyberGear_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 500);
}

void CyberGear_Motor::read(uint32_t header, uint8_t * data, uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;
  feedback_communication_type = ((header) >> 24) & 0x1F;  //先读取帧类型，只取低五位

  //正常工作时的反馈
  if (feedback_communication_type == Communication_Type_MotorRequest) {
    this->feedback_master_id = header & 0xFF;    //指明这一帧发给哪台主机的(取低8位)
    this->feedback_can_id = header >> 8 & 0xFF;  //指明这一帧是哪个电机发的(取bit 8-15)
    this->status_error = (header >> 16) & 0x3F;  //只取低六位
    this->mode = (header >> 22) & 0x03;          //只取低两位
    this->angle = uint_to_float(data[0] << 8 | data[1], -pmax_, pmax_, 16);
    this->speed = uint_to_float(data[2] << 8 | data[3], -vmax_, vmax_, 16);
    this->torque = uint_to_float(data[4] << 8 | data[5], -tmax_, tmax_, 16);
    this->temperature = float((data[6] << 8 | data[7]) & 0xFFFF) / 10.0f;
  }
  //故障反馈
  else if (feedback_communication_type == Communication_Type_ErrorFeedback) {
    this->feedback_can_id = header & 0xFF;                                                // 取低8位
    this->feedback_master_id = (header >> 8) & 0xFF;                                      // 取低8位
    this->error = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) & 0x0001FFFF;  //取低17位
    this->temp_warning = (data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]) & 0xFFFFFFFF;
  }
  //单个参数读取的反馈
  else if (feedback_communication_type == Communication_Type_GetSingleParameter) {
    this->feedback_can_id = header & 0xFF;            // 取低8位
    this->feedback_master_id = (header >> 8) & 0xFF;  // 取低8位
    this->index = (data[1] << 8 | data[0]) & 0xFFFF;
    if (this->index == 0x7005) {
      this->parameter = (int8_t)(data[4] & 0xFF);
    }
    else {
      this->parameter = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
    }
  }
  //获取设备ID的反馈
  else if (feedback_communication_type == Communication_Type_GetID) {
    this->feedback_can_id = (header >> 8) & 0xFF;  // 取低8位
    this->ID_gotten = (header & 0xFF) == 0xFE;     //读到0xFE说明获取ID成功
    // 组装64位MCU_ID：必须先使用static_cast提升为uint64_t再移位，避免32位提升后左移>31导致未定义行为
    this->MCU_ID = (static_cast<uint64_t>(data[0]) << 56) | (static_cast<uint64_t>(data[1]) << 48) |
                   (static_cast<uint64_t>(data[2]) << 40) | (static_cast<uint64_t>(data[3]) << 32) |
                   (static_cast<uint64_t>(data[4]) << 24) | (static_cast<uint64_t>(data[5]) << 16) |
                   (static_cast<uint64_t>(data[6]) << 8) | (static_cast<uint64_t>(data[7]));
  }
}

//CyberGear的一堆命令（真的多，不一定都用得到
//运动控制模式，只发送力矩，不使用目标角度、角速度、Kp、Kd
void CyberGear_Motor::cmd(float torque)
{
  communication_type = Communication_Type_MotionControl;
  tar_torque = float_to_uint(torque, -tmax_, tmax_, 16);
  tar_position_ = float_to_uint(0, -4 * sp::SP_PI, 4 * sp::SP_PI, 16);
  tar_speed_ = float_to_uint(0, -30.0f, 30.0f, 16);
  Kp_ = float_to_uint(0, 0.0f, 500.0f, 16);
  Kd_ = float_to_uint(0, 0.0f, 5.0f, 16);
}
//获取设备ID命令
void CyberGear_Motor::cmd_get_id() { communication_type = Communication_Type_GetID; }
//设置电机CAN ID命令
void CyberGear_Motor::cmd_set_can_id() { communication_type = Communication_Type_CanID; }
//使能电机命令
void CyberGear_Motor::cmd_motor_enable() { communication_type = Communication_Type_MotorEnable; }
//停止电机命令
void CyberGear_Motor::cmd_motor_stop() { communication_type = Communication_Type_MotorStop; }
//设置当前位置为零点命令
void CyberGear_Motor::cmd_set_pos_zero() { communication_type = Communication_Type_SetPosZero; }
//读取单个参数命令
void CyberGear_Motor::cmd_get_single_parameter(uint16_t index)
{
  communication_type = Communication_Type_GetSingleParameter;
  index_ = index;
}
//设置单个参数命令
void CyberGear_Motor::cmd_set_single_parameter(uint16_t index, float parameter)
{
  communication_type = Communication_Type_SetSingleParameter;
  index_ = index;
  parameter_ = parameter;
}

//把能写到data里的内容传到can发送数据中
void CyberGear_Motor::write(uint8_t * data) const
{
  if (communication_type == Communication_Type_MotionControl) {
    // data[0] = 0;
    // data[1] = 0;
    // data[2] = 0;
    // data[3] = 0;
    // data[4] = 0;
    // data[5] = 0;
    // data[6] = 0;
    // data[7] = 0;
    //理论上这里发的都是0
    data[0] = tar_position_ >> 8 & 0xFF;
    data[1] = tar_position_ & 0xFF;
    data[2] = tar_speed_ >> 8 & 0xFF;
    data[3] = tar_speed_ & 0xFF;
    data[4] = Kp_ >> 8 & 0xFF;
    data[5] = Kp_ & 0xFF;
    data[6] = Kd_ >> 8 & 0xFF;
    data[7] = Kd_ & 0xFF;
  }
  else if (
    communication_type == Communication_Type_GetID ||
    communication_type == Communication_Type_MotorEnable ||
    communication_type == Communication_Type_CanID) {
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  else if (
    communication_type == Communication_Type_MotorStop ||
    communication_type == Communication_Type_SetPosZero) {
    data[0] = 0x01;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  else if (communication_type == Communication_Type_GetSingleParameter) {
    data[0] = index_ & 0xFF;
    data[1] = (index_ >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  else if (communication_type == Communication_Type_SetSingleParameter) {
    data[0] = index_ & 0xFF;
    data[1] = (index_ >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    if (index_ != run_mode) {
      data[4] = parameter_ >> 24 & 0xFF;
      data[5] = (parameter_ >> 16) & 0xFF;
      data[6] = (parameter_ >> 8) & 0xFF;
      data[7] = parameter_ & 0xFF;
    }
    else if (index_ == run_mode) {
      data[4] = (uint8_t)(parameter_);
      data[5] = 0x00;
      data[6] = 0x00;
      data[7] = 0x00;
    }
  }
}

}  // namespace sp
