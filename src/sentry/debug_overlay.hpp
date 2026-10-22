#ifndef STANDARD__DEBUG_OVERLAY_HPP
#define STANDARD__DEBUG_OVERLAY_HPP

#include <cstddef>
#include <list>

#include <opencv2/opencv.hpp>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/tracker.hpp"

namespace sentry_app
{
struct DebugOverlayInput
{
  int frame_count = 0;
  double bullet_speed = 0.0;
  io::Mode mode = io::Mode::idle;
  std::size_t armors_before_tracker = 0;
  std::size_t armors_after_tracker = 0;
  const io::Command & command;
  const std::list<auto_aim::Armor> & draw_armors;
  const std::list<auto_aim::Target> & targets;
  const auto_aim::Tracker & tracker;
  const auto_aim::Aimer & aimer;
  const auto_aim::Shooter & shooter;
};

bool show_debug_overlay(
  const cv::Mat & img, auto_aim::Solver & solver, const DebugOverlayInput & input);

}  // namespace sentry_app

#endif  // STANDARD__DEBUG_OVERLAY_HPP
