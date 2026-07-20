/**
 * @file    slip_detect.hpp
 * @brief   舵轮底盘滑移率检测模块
 *
 * 通过比较陀螺仪真实角速度与轮式里程计推算角速度，
 * 估计每个轮子的滑移率，为功率分配提供惩罚系数。
 *
 * 核心原理：
 *   λ_i = (v_wheel_i - v_ground_i) / max(|v_wheel_i|, |v_ground_i|, ε)
 *   其中 v_ground_i 使用陀螺仪真实 wz 投影计算
 *
 * 轮序约定（与 chassis_para.hpp 中 move_t 一致）：
 *   [0]=RF(右前), [1]=LF(左前), [2]=LR(左后), [3]=RR(右后)
 */

#ifndef SP__SLIP_DETECT_HPP
#define SP__SLIP_DETECT_HPP

namespace sp
{

/**
 * @brief 单个轮子的滑移数据
 */
struct WheelSlipData
{
  float slip_ratio;   ///< 滑移率 [-1, 1]  正=驱动空转, 负=制动抱死
  float slip_penalty; ///< 功率惩罚系数 [0, 1]  1=无惩罚, ~0=几乎切断
  float v_wheel;      ///< 轮子线速度 (m/s)  来自编码器
  float v_ground;     ///< 地面真实速度投影 (m/s)  来自IMU+运动学
};

class SlipDetect
{
public:
  /**
   * @param half_length   底盘前后轮距的一半 (m)
   * @param half_width    底盘左右轮距的一半 (m)
   * @param alpha_slip    滑移率低通滤波系数 (0~1), 越大响应越快但越噪声敏感
   * @param penalty_gain  惩罚增益 (>0), 越大对滑移的惩罚越激进
   * @param slip_threshold 滑移判定阈值, |λ|超过此值认为打滑
   */
  SlipDetect(
    float half_length, float half_width, float alpha_slip = 0.08f,
    float penalty_gain = 1.5f, float slip_threshold = 0.25f);

  // ---- 只读输出 ----
  WheelSlipData wheel_slip[4]; ///< 四轮滑移数据 [RF, LF, LR, RR]
  bool any_slipping;           ///< 是否有任意轮子在打滑
  float avg_slip_ratio;        ///< 四轮平均绝对滑移率 [0, 1]
  float slip_wz;               ///< 角速度滑移量: chassis.wz - imu_wz (rad/s)

  /**
   * @brief 核心更新函数 —— 每个控制周期调用一次
   *
   * @param imu_wz         陀螺仪真实Z轴角速度 (rad/s)  — imu.vyaw 或 imu.w[2]
   * @param chassis_vx     轮式里程计X轴线速度 (m/s)    — chassis.vx
   * @param chassis_vy     轮式里程计Y轴线速度 (m/s)    — chassis.vy
   * @param wheel_speeds   四轮编码器转速 (rad/s)       — move_data.current_drive_speed
   * @param wheel_angles   四轮当前舵角 (rad)           — move_data.current_pivot_angle
   * @param wheel_radius   轮子半径 (m)
   */
  void update(
    float imu_wz, float chassis_vx, float chassis_vy, const float wheel_speeds[4],
    const float wheel_angles[4], float wheel_radius);

  /**
   * @brief 对功率分配权重施加滑移惩罚并重新归一化
   *
   * 用法：在 HKUST 算法算出 K_original[4] 之后、解二次方程之前调用。
   * 不打滑的轮子 K 值几乎不变，打滑轮 K 值被压缩，释放的权重自动流向抓地轮。
   *
   * @param K_original   HKUST 算法原始分配权重 [4]
   * @param K_corrected  施加惩罚并归一化后的修正权重 [4]
   */
  void apply_slip_penalty(const float K_original[4], float K_corrected[4]);

  /**
   * @brief 获取当前整体滑移限幅系数（用于全部驱动轮统一降功率）
   * @return [0.3, 1.0]  1.0=不限幅, 越小限幅越狠
   */
  float get_global_slip_scale() const;

private:
  // 底盘几何
  const float l_, w_;

  // 滤波与增益参数
  const float alpha_slip_;
  const float penalty_gain_;
  const float slip_threshold_;

  // 初始化标志
  bool inited_;

  // 内部低通状态
  float filtered_slip_[4];
};

}  // namespace sp

#endif  // SP__SLIP_DETECT_HPP
