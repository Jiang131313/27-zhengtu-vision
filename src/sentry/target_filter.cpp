#include "src/sentry/target_filter.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>

#include "tools/logger.hpp"

namespace sentry_app
{
namespace
{
std::string normalized(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}
}  // namespace

auto_aim::ArmorName parse_armor_name(const std::string & armor_name)
{
  static const std::unordered_map<std::string, auto_aim::ArmorName> kArmorNames = {
    {"hero", auto_aim::ArmorName::one},
    {"one", auto_aim::ArmorName::one},
    {"1", auto_aim::ArmorName::one},
    {"two", auto_aim::ArmorName::two},
    {"2", auto_aim::ArmorName::two},
    {"engineer", auto_aim::ArmorName::two},
    {"three", auto_aim::ArmorName::three},
    {"3", auto_aim::ArmorName::three},
    {"infantry3", auto_aim::ArmorName::three},
    {"four", auto_aim::ArmorName::four},
    {"4", auto_aim::ArmorName::four},
    {"infantry4", auto_aim::ArmorName::four},
    {"five", auto_aim::ArmorName::five},
    {"5", auto_aim::ArmorName::five},
    {"infantry5", auto_aim::ArmorName::five},
    {"sentry", auto_aim::ArmorName::sentry},
    {"outpost", auto_aim::ArmorName::outpost},
    {"base", auto_aim::ArmorName::base},
  };

  const auto key = normalized(armor_name);
  const auto it = kArmorNames.find(key);
  if (it != kArmorNames.end()) return it->second;

  tools::logger()->warn("Unknown priority_target '{}', fallback to hero.", armor_name);
  return auto_aim::ArmorName::one;
}

bool is_enemy_priority_target(
  const auto_aim::Armor & armor, auto_aim::Color enemy_color,
  auto_aim::ArmorName priority_target)
{
  return armor.color == enemy_color && armor.name == priority_target;
}

bool contains_enemy_priority_target(
  const std::list<auto_aim::Armor> & armors, auto_aim::Color enemy_color,
  auto_aim::ArmorName priority_target)
{
  return std::any_of(armors.begin(), armors.end(), [&](const auto_aim::Armor & armor) {
    return is_enemy_priority_target(armor, enemy_color, priority_target);
  });
}

bool filter_priority_target(
  std::list<auto_aim::Armor> & armors, auto_aim::Color enemy_color,
  auto_aim::ArmorName priority_target)
{
  // 动态优先目标策略：
  // 只要这一帧检测结果里出现了配置指定的敌方兵种，就在进入 Tracker 之前
  // 先把其他目标全部滤掉，只把这个优先目标交给后续逻辑。
  if (!contains_enemy_priority_target(armors, enemy_color, priority_target)) return false;

  armors.remove_if([&](const auto_aim::Armor & armor) {
    return !is_enemy_priority_target(armor, enemy_color, priority_target);
  });
  return true;
}

}  // namespace sentry_app
