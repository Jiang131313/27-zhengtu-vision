#ifndef STANDARD__CONFIG_HPP
#define STANDARD__CONFIG_HPP

#include <Eigen/Dense>

#include <string>

#include "tasks/auto_aim/armor.hpp"

namespace sentry_app
{
struct SentryConfig
{
  bool show_display = true;
  auto_aim::Color enemy_color = auto_aim::Color::blue;
  auto_aim::ArmorName priority_target = auto_aim::ArmorName::one;
  bool imu_enabled = true;
  std::string imu_port = "/dev/ttyACM0";
  int imu_baudrate = 921600;
  // Kalibr 时间偏移，单位 s；t_imu = t_cam + imu_time_offset_s。
  double imu_time_offset_s = 0.0;
  // 枪口发射点在云台坐标系下的位置，单位 m；x 前、y 左、z 上。
  Eigen::Vector3d t_muzzle2gimbal = Eigen::Vector3d::Zero();
};

SentryConfig load_config(const std::string & config_path);

}  // namespace sentry_app

#endif  // STANDARD__CONFIG_HPP
