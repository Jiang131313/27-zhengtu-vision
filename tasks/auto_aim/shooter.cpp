#include "shooter.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  // 上游同济 shooter.cpp 把 first/second_tolerance 当「弧度」用（/57.3），
  // 因为它比较的 command.yaw 是世界系绝对角（弧度）。
  // 本工程 command_builder 下发的 command.yaw/pitch 是「相对当前枪管」的角度、
  // 单位是「度」，所以容差直接用「度」，不再 /57.3。
  first_tolerance_ = yaml["first_tolerance"].as<double>();    // degree
  second_tolerance_ = yaml["second_tolerance"].as<double>();  // degree
  small_gyro_fire_yaw_thresh_ =
    yaml["small_gyro_fire_yaw_thresh"] ? yaml["small_gyro_fire_yaw_thresh"].as<double>() : 1.0;
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  use_gimbal_feedback_ =
    yaml["use_gimbal_feedback"] ? yaml["use_gimbal_feedback"].as<bool>() : true;
}

void Shooter::reset_debug_state()
{
  debug_small_gyro_yaw_error_deg = std::nullopt;
  debug_small_gyro_fire_window = false;
  debug_small_gyro_checked = false;
}

bool Shooter::shoot_small_gyro(const auto_aim::Aimer & aimer, const auto_aim::Target & target)
{
  if (!aimer.debug_small_gyro_active || !aimer.debug_aim_point.valid) return false;
  if (aimer.debug_small_gyro_armor_id < 0 || aimer.debug_total_predict_time <= 0) return false;

  debug_small_gyro_checked = true;

  auto predicted_target = target;
  predicted_target.predict(aimer.debug_total_predict_time);
  auto predicted_armors = predicted_target.armor_xyza_list();
  if (aimer.debug_small_gyro_armor_id >= static_cast<int>(predicted_armors.size())) return false;

  const auto & predicted_armor = predicted_armors[aimer.debug_small_gyro_armor_id];
  const double predicted_yaw = std::atan2(predicted_armor[1], predicted_armor[0]);
  const double aim_yaw = std::atan2(aimer.debug_aim_point.xyza[1], aimer.debug_aim_point.xyza[0]);
  const double yaw_error_deg =
    std::abs(tools::limit_rad(predicted_yaw - aim_yaw)) * 180.0 / 3.141592653589793;
  debug_small_gyro_yaw_error_deg = yaw_error_deg;
  debug_small_gyro_fire_window = yaw_error_deg < small_gyro_fire_yaw_thresh_;

  return debug_small_gyro_fire_window;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  reset_debug_state();
  if (!command.control || targets.empty() || !auto_fire_) return false;

  // 目标位置（m）；距离越远，容差越大（单位：度）
  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;

  // ===== 火控判据：同济上游 shooter.cpp 原版三条 =====
  // 上游（世界系绝对角，弧度）：
  //   |last_command_.yaw - command.yaw| < tolerance * 2    // command 突变时不射击
  //   |gimbal_pos[0]     - last_command_.yaw| < tolerance  // 云台已经跟到上一次指令
  //   aimer.debug_aim_point.valid
  //
  // 本工程的 command.yaw 已经是「相对当前枪管」的角度，也就是
  // 「云台当前指向」与「期望指向」之间的残差：
  //   command.yaw = 期望世界系 yaw - 当前云台世界系 yaw
  // 所以上游第二条 |云台角 - 上一次期望角| 在这里就等价于 |command.yaw|。
  (void)gimbal_pos;  // command.yaw 里已经含了云台反馈，不需要世界系云台角
  const bool command_ok = std::abs(last_command_.yaw - command.yaw) < tolerance * 2;
  const bool gimbal_ok = !use_gimbal_feedback_ || (std::abs(command.yaw) < tolerance);
  bool fire_ready = command_ok && gimbal_ok && aimer.debug_aim_point.valid;

  // 小陀螺：在以上基础上再加一层「预测装甲板是否已进入枪口 yaw 窗口」。
  // 放在后面算，是为了让 debug 面板在小陀螺模式下始终有 yaw_err 显示。
  if (aimer.debug_small_gyro_active) {
    const bool window_open = shoot_small_gyro(aimer, targets.front());
    fire_ready = fire_ready && window_open;
  }

  last_command_ = command;
  return fire_ready;
}

}  // namespace auto_aim
