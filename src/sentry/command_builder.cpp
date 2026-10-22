#include "src/sentry/command_builder.hpp"

#include <optional>

#include "tools/math_tools.hpp"

namespace sentry_app
{
namespace
{
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

// ---------------------------------------------------------------------------
// 通用工具
// ---------------------------------------------------------------------------

io::Command disabled_command()
{
  return {false, false, 0.0, 0.0};
}

bool should_disable(
  const io::Command & aimer_command, const std::list<auto_aim::Target> & targets,
  const auto_aim::Aimer & aimer)
{
  return !aimer_command.control || targets.empty() || !aimer.debug_aim_point.valid;
}

// ---------------------------------------------------------------------------
// 枪管系几何：把 aimer 解出来的世界系瞄准角转成「相对当前枪管」的 (yaw, pitch)，弧度。
//
// aimer_command.yaw/.pitch 是世界系绝对角，且 pitch 抬头为正
// （见 tools/trajectory.hpp: double pitch; // 抬头为正），
// 与 tools::ypd2xyz 是同一套约定（z = d*sin(pitch)，抬头为正）。
//
// 所以先用 ypd2xyz 把角度还原成方向向量，再左乘 R_gimbal2world^T 转进枪管系，
// 最后用 xyz2ypd 取角 —— 和原来「检测值直瞄」那条路径用的是同一对函数，
// 下位机收到的角度约定完全不变，只是角度从「本帧检测直瞄」换成了
// 「aimer 的弹道解」（含提前量 + 下坠补偿 + yaw_offset/pitch_offset）。
// ---------------------------------------------------------------------------

std::optional<Eigen::Vector2d> aimer_barrel_aim(
  const io::Command & aimer_command, const auto_aim::Solver & solver)
{
  const Eigen::Vector3d dir_world =
    tools::ypd2xyz(Eigen::Vector3d{aimer_command.yaw, aimer_command.pitch, 1.0});
  if (dir_world.squaredNorm() < 1e-9) return std::nullopt;
  const Eigen::Vector3d dir_barrel = solver.R_gimbal2world().transpose() * dir_world;
  return tools::xyz2ypd(dir_barrel).head<2>();
}

// ---------------------------------------------------------------------------
// 算出下发给下位机的角度（度）
// ---------------------------------------------------------------------------
struct CommandAngles
{
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
};

std::optional<CommandAngles> resolve_command_angles(
  const io::Command & aimer_command, const auto_aim::Solver & solver)
{
  const auto aim = aimer_barrel_aim(aimer_command, solver);
  if (!aim.has_value()) return std::nullopt;

  CommandAngles angles;
  angles.yaw_deg = aim->x() * kRad2Deg;
  angles.pitch_deg = aim->y() * kRad2Deg;
  return angles;
}

}  // namespace

// ---------------------------------------------------------------------------
// public entry: 火控指令（barrel_relative，相对当前枪管）
// ---------------------------------------------------------------------------

io::Command build_auto_aim_command(
  const io::Command & aimer_command, const std::list<auto_aim::Target> & targets,
  const std::list<auto_aim::Armor> & armors, const auto_aim::Aimer & aimer,
  const auto_aim::Solver & solver, const SentryConfig & config)
{
  (void)armors;
  (void)config;

  if (should_disable(aimer_command, targets, aimer)) return disabled_command();

  // 不再要求本帧必须匹配到装甲板。temp_lost 期间 targets 非空、EKF 仍在预测、
  // aimer 也已经把预测点算好；原来这里直接 disabled_command() 会把云台停死，
  // 目标立刻飞出视野，max_temp_lost_count 的缓冲白烧，最后变成真 lost。
  const auto angles = resolve_command_angles(aimer_command, solver);
  if (!angles.has_value()) return disabled_command();

  auto command = aimer_command;
  command.control = true;
  command.yaw = angles->yaw_deg;
  command.pitch = angles->pitch_deg;
  return command;
}

}  // namespace sentry_app
