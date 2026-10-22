#ifndef STANDARD__TARGET_FILTER_HPP
#define STANDARD__TARGET_FILTER_HPP

#include <list>
#include <string>

#include "tasks/auto_aim/armor.hpp"

namespace sentry_app
{
auto_aim::ArmorName parse_armor_name(const std::string & armor_name);

bool is_enemy_priority_target(
  const auto_aim::Armor & armor, auto_aim::Color enemy_color,
  auto_aim::ArmorName priority_target);

bool contains_enemy_priority_target(
  const std::list<auto_aim::Armor> & armors, auto_aim::Color enemy_color,
  auto_aim::ArmorName priority_target);

bool filter_priority_target(
  std::list<auto_aim::Armor> & armors, auto_aim::Color enemy_color,
  auto_aim::ArmorName priority_target);

}  // namespace sentry_app

#endif  // STANDARD__TARGET_FILTER_HPP
