#include "src/sentry/config.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "src/sentry/target_filter.hpp"

namespace sentry_app
{
namespace
{
auto_aim::Color parse_enemy_color(const YAML::Node & yaml)
{
  const auto enemy_color = yaml["enemy_color"] ? yaml["enemy_color"].as<std::string>() : "blue";
  return enemy_color == "red" ? auto_aim::Color::red : auto_aim::Color::blue;
}

Eigen::Vector3d parse_t_muzzle2gimbal(const YAML::Node & yaml)
{
  if (!yaml["t_muzzle2gimbal"]) return Eigen::Vector3d::Zero();

  const auto data = yaml["t_muzzle2gimbal"].as<std::vector<double>>();
  if (data.size() != 3) {
    throw std::runtime_error("t_muzzle2gimbal must contain exactly 3 values.");
  }

  return Eigen::Vector3d(data[0], data[1], data[2]);
}
}  // namespace

SentryConfig load_config(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);

  SentryConfig config;
  config.show_display = yaml["display"] ? yaml["display"].as<bool>() : true;
  // 这里显式读取敌方颜色，是为了在 sentry 主链路里做“英雄优先”筛选。
  // 这次需求是跟着 fuck.sh 的真实启动路径走：
  //   fuck.sh -> build/sentry -> configs/demo.yaml -d
  // 所以只在 sentry 里补这一层逻辑，不去改 tracker / aimer / 其他模式的共用行为。
  config.enemy_color = parse_enemy_color(yaml);
  config.priority_target =
    parse_armor_name(yaml["priority_target"] ? yaml["priority_target"].as<std::string>() : "hero");
  config.imu_enabled = yaml["imu_enabled"] ? yaml["imu_enabled"].as<bool>() : true;
  config.imu_port =
    yaml["imu_port"] ? yaml["imu_port"].as<std::string>() : std::string("/dev/ttyACM0");
  config.imu_baudrate = yaml["imu_baudrate"] ? yaml["imu_baudrate"].as<int>() : 921600;
  config.imu_time_offset_s =
    yaml["imu_time_offset_s"] ? yaml["imu_time_offset_s"].as<double>() : 0.0;
  config.t_muzzle2gimbal = parse_t_muzzle2gimbal(yaml);

  return config;
}

}  // namespace sentry_app
