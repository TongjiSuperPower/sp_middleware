/**
 * @file    slip_detect.cpp
 * @brief   滑移率检测模块实现
 *
 * 滑移率定义:
 *   λ_i = (v_wheel_i - v_ground_i) / max(|v_wheel_i|, |v_ground_i|, ε)
 *
 *   其中:
 *     v_wheel_i = ω_i × r            — 编码器推算的轮子线速度
 *     v_ground_i = V_chassis · e_i + ωz_true × (r_pos × e_i)
 *                = (vx - ωz·y_i)·cosθ_i + (vy + ωz·x_i)·sinθ_i
 *                — 底盘真实运动在轮子滚动方向上的投影
 *
 * 惩罚函数:
 *   slip_penalty = clamp(1 - penalty_gain × |λ|, 0.05, 1.0)
 */

#include "slip_detect.hpp"

#include <algorithm>
#include <cmath>

namespace sp
{

SlipDetect::SlipDetect(
  float half_length, float half_width, float alpha_slip, float penalty_gain, float slip_threshold)
: l_(half_length),
  w_(half_width),
  alpha_slip_(alpha_slip),
  penalty_gain_(penalty_gain),
  slip_threshold_(slip_threshold),
  any_slipping(false),
  avg_slip_ratio(0.0f),
  slip_wz(0.0f),
  inited_(false)
{
  for (int i = 0; i < 4; i++) {
    wheel_slip[i] = {0.0f, 1.0f, 0.0f, 0.0f};
    filtered_slip_[i] = 0.0f;
  }
}

void SlipDetect::update(
  float imu_wz, float chassis_vx, float chassis_vy, const float wheel_speeds[4],
  const float wheel_angles[4], float wheel_radius)
{
  // ---- 轮位坐标 (车体坐标系: X向前, Y向左) ----
  //   [0]=RF: (x=+l, y=-w)
  //   [1]=LF: (x=+l, y=+w)
  //   [2]=LR: (x=-l, y=+w)
  //   [3]=RR: (x=-l, y=-w)
  const float px[4] = {+l_, +l_, -l_, -l_};
  const float py[4] = {-w_, +w_, +w_, -w_};

  // ---- 角速度滑移量 (整体指标) ----
  // chassis_wz 没有直接传入，从轮速估算意义不大，
  // 这里用 imu_wz 和轮速的差异来估计
  // 注意：slip_wz 需要外部配合 chassis.wz 来计算
  // 这里保留为整体观测指标

  float slip_sum = 0.0f;
  int slip_count = 0;
  any_slipping = false;

  for (int i = 0; i < 4; i++) {
    // --- 1. 轮子线速度 (编码器) ---
    float v_wheel = wheel_speeds[i] * wheel_radius;
    wheel_slip[i].v_wheel = v_wheel;

    // --- 2. 底盘在轮位处的真实线速度 ---
    //     v_at_wheel_x = vx - ωz × y_i   (刚体速度场)
    //     v_at_wheel_y = vy + ωz × x_i
    float v_gnd_x = chassis_vx - imu_wz * py[i];
    float v_gnd_y = chassis_vy + imu_wz * px[i];

    // --- 3. 投影到轮子滚动方向 ---
    float cos_a = std::cos(wheel_angles[i]);
    float sin_a = std::sin(wheel_angles[i]);
    float v_ground = v_gnd_x * cos_a + v_gnd_y * sin_a;
    wheel_slip[i].v_ground = v_ground;

    // --- 4. 滑移率计算 ---
    float max_v = std::max(std::abs(v_wheel), std::abs(v_ground));
    float raw_slip = 0.0f;

    // 低速死区: 速度太低时滑移率无意义 (噪声主导)
    constexpr float kSpeedDeadZone = 0.05f;  // 5 cm/s
    if (max_v > kSpeedDeadZone) {
      raw_slip = (v_wheel - v_ground) / max_v;
      // 钳制到 [-1, 1]
      if (raw_slip > 1.0f) raw_slip = 1.0f;
      if (raw_slip < -1.0f) raw_slip = -1.0f;
    }

    // --- 5. 低通滤波 ---
    if (!inited_) {
      filtered_slip_[i] = raw_slip;
    }
    else {
      filtered_slip_[i] =
        alpha_slip_ * raw_slip + (1.0f - alpha_slip_) * filtered_slip_[i];
    }
    wheel_slip[i].slip_ratio = filtered_slip_[i];

    // --- 6. 惩罚系数 ---
    float abs_slip = std::abs(filtered_slip_[i]);
    wheel_slip[i].slip_penalty = 1.0f - penalty_gain_ * abs_slip;
    // 钳制: 不完全切断功率，保留最低 5%
    if (wheel_slip[i].slip_penalty < 0.05f) {
      wheel_slip[i].slip_penalty = 0.05f;
    }
    if (wheel_slip[i].slip_penalty > 1.0f) {
      wheel_slip[i].slip_penalty = 1.0f;
    }

    // --- 7. 整体状态 ---
    if (abs_slip > slip_threshold_) {
      any_slipping = true;
    }
    slip_sum += abs_slip;
    slip_count++;
  }

  inited_ = true;
  avg_slip_ratio = (slip_count > 0) ? (slip_sum / static_cast<float>(slip_count)) : 0.0f;
}

void SlipDetect::apply_slip_penalty(const float K_original[4], float K_corrected[4])
{
  // 每个轮子的权重乘以对应的滑移惩罚系数
  for (int i = 0; i < 4; i++) {
    K_corrected[i] = K_original[i] * wheel_slip[i].slip_penalty;
  }

  // 重新归一化: 释放的功率额度按比例自动流向不打滑的轮子
  float sum = K_corrected[0] + K_corrected[1] + K_corrected[2] + K_corrected[3];

  if (sum > 1e-6f) {
    float inv_sum = 1.0f / sum;
    for (int i = 0; i < 4; i++) {
      K_corrected[i] *= inv_sum;
    }
  }
  else {
    // 所有轮子都严重打滑 → 均分，让每个轮子至少有一些基础力矩维持姿态
    for (int i = 0; i < 4; i++) {
      K_corrected[i] = 0.25f;
    }
  }
}

float SlipDetect::get_global_slip_scale() const
{
  // 平均滑移率超过阈值时，输出整体限幅系数
  // 映射: avg_slip > 0.3 开始限幅, avg_slip = 0.8 时限到 0.3
  if (avg_slip_ratio < slip_threshold_) {
    return 1.0f;
  }

  // 线性插值: [threshold, 0.8] → [1.0, 0.3]
  float t = (avg_slip_ratio - slip_threshold_) / (0.8f - slip_threshold_);
  if (t > 1.0f) t = 1.0f;

  float scale = 1.0f - t * 0.7f;  // 最多降到 0.3
  return scale;
}

}  // namespace sp
