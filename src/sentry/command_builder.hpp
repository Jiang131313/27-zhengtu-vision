#ifndef STANDARD__COMMAND_BUILDER_HPP
#define STANDARD__COMMAND_BUILDER_HPP

#include <list>

#include "io/command.hpp"
#include "src/sentry/config.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"

namespace sentry_app
{

io::Command build_auto_aim_command(
  const io::Command & aimer_command, const std::list<auto_aim::Target> & targets,
  const std::list<auto_aim::Armor> & armors, const auto_aim::Aimer & aimer,
  const auto_aim::Solver & solver, const SentryConfig & config);

}  // namespace sentry_app

#endif  // STANDARD__COMMAND_BUILDER_HPP
