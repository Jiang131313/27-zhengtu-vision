#include "src/sentry/debug_overlay.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "tools/img_tools.hpp"

namespace sentry_app
{
namespace
{
void blend_panel(cv::Mat & vis, int width, int height)
{
  if (vis.empty()) return;

  const int panel_w = std::min(width, vis.cols);
  const int panel_h = std::min(height, vis.rows);
  if (panel_w <= 0 || panel_h <= 0) return;

  // 半透明黑色背景，保证 Tracker/Estimator 文字在任何画面下都清晰可见。
  cv::Mat vis_roi = vis(cv::Rect(0, 0, panel_w, panel_h));
  cv::Mat panel(panel_h, panel_w, vis.type(), cv::Scalar(0, 0, 0));
  cv::Mat blended;
  cv::addWeighted(panel, 0.5, vis_roi, 0.5, 0, blended);
  blended.copyTo(vis_roi);
}

void draw_detection_boxes(cv::Mat & vis, const std::list<auto_aim::Armor> & draw_armors)
{
  // 检测框：用与 detection 窗口同一份 armors 画，保证两边一致。
  const int vis_w = vis.cols;
  const int vis_h = vis.rows;

  for (const auto & armor : draw_armors) {
    if (armor.points.size() < 4) continue;

    std::vector<cv::Point> points;
    for (const auto & point : armor.points) {
      const int px = std::clamp(cvRound(point.x), 0, vis_w - 1);
      const int py = std::clamp(cvRound(point.y), 0, vis_h - 1);
      points.emplace_back(px, py);
    }

    if (points.size() >= 4) {
      std::vector<std::vector<cv::Point>> contours = {points};
      cv::polylines(vis, contours, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }
  }
}

void draw_orbit(
  cv::Mat & vis, auto_aim::Solver & solver, const Eigen::VectorXd & x, double radius,
  double z_world, const cv::Scalar & color)
{
  if (radius <= 1e-3) return;

  std::vector<cv::Point> segment;
  for (int i = 0; i <= 72; ++i) {
    const double theta = (2.0 * CV_PI * i) / 72.0;
    const cv::Point3f world_point(
      static_cast<float>(x[0] + radius * std::cos(theta)),
      static_cast<float>(x[2] + radius * std::sin(theta)), static_cast<float>(z_world));
    const auto pixels = solver.world2pixel({world_point});
    if (!pixels.empty()) {
      const cv::Point pixel(cvRound(pixels[0].x), cvRound(pixels[0].y));
      if (pixel.x >= 0 && pixel.x < vis.cols && pixel.y >= 0 && pixel.y < vis.rows) {
        segment.push_back(pixel);
        continue;
      }
    }

    if (segment.size() >= 2) {
      std::vector<std::vector<cv::Point>> contours = {segment};
      cv::polylines(vis, contours, false, color, 1, cv::LINE_AA);
    }
    segment.clear();
  }

  if (segment.size() >= 2) {
    std::vector<std::vector<cv::Point>> contours = {segment};
    cv::polylines(vis, contours, false, color, 1, cv::LINE_AA);
  }
}

void draw_text_panel(cv::Mat & vis, const DebugOverlayInput & input)
{
  int line_h = 28;
  int y0 = std::max(30, vis.rows - 560);

  // 帧号（用于确认画面在更新）。
  tools::draw_text(vis, fmt::format("frame: {}", input.frame_count), {10, y0}, {0, 255, 255}, 0.7);
  y0 += line_h;

  // ----- 跟踪器 -----
  tools::draw_text(vis, "Tracker:", {10, y0}, {0, 255, 255}, 0.7);
  y0 += line_h;
  tools::draw_text(
    vis, fmt::format("  state: [{}]", input.tracker.state()), {10, y0}, {255, 255, 255}, 0.6);
  y0 += line_h;
  tools::draw_text(
    vis,
    fmt::format(
      "  armors: {} (enemy: {})  targets: {}", input.armors_before_tracker,
      input.armors_after_tracker, input.targets.size()),
    {10, y0}, {255, 255, 255}, 0.6);
  y0 += line_h;

  if (
    input.armors_before_tracker > 0 && input.armors_after_tracker == 0 &&
    !input.draw_armors.empty())
  {
    // 有绿框但跟踪器不触发：enemy_color 过滤掉了所有检测，给出可操作提示。
    const auto & first = input.draw_armors.front();
    const std::string det_color =
      first.color <= auto_aim::Color::purple
        ? auto_aim::COLORS[static_cast<std::size_t>(first.color)]
        : "?";
    tools::draw_text(
      vis, fmt::format("  >> config enemy_color to \"{}\" (now filtered to 0)", det_color),
      {10, y0}, {0, 200, 255}, 0.55);
    y0 += line_h;
  }

  // ----- 整车估计器 11维 (EKF) -----
  /*
  旋转中心位置：x y z
  旋转中心速度：vx vy vz
  车体角度：a
  角速度：w
  主半径：r
  长短轴差：l
  高低差：h
  */
  tools::draw_text(vis, "Estimator:", {10, y0}, {0, 255, 255}, 0.7);
  y0 += line_h;
  if (!input.targets.empty()) {
    const auto & target = input.targets.front();
    tools::draw_text(
      vis, fmt::format("  armor_id: {}", target.last_id), {10, y0}, {255, 255, 255}, 0.6);
    y0 += line_h;

    const Eigen::VectorXd x = target.ekf_x();
    if (x.size() >= 11) {
      tools::draw_text(
        vis, fmt::format("  xyz: ({:.2f},{:.2f},{:.2f})", x[0], x[2], x[4]), {10, y0},
        {200, 200, 255}, 0.6);
      y0 += line_h;
      const double speed = std::sqrt(x[1] * x[1] + x[3] * x[3] + x[5] * x[5]);
      tools::draw_text(
        vis,
        fmt::format("  vel: ({:.2f},{:.2f},{:.2f}) |v|:{:.2f}", x[1], x[3], x[5], speed),
        {10, y0}, {200, 200, 255}, 0.6);
      y0 += line_h;
      tools::draw_text(
        vis, fmt::format("  angular_w: {:.2f} rad/s", x[7]), {10, y0},
        {200, 200, 255}, 0.6);
      y0 += line_h;

      const std::string gyro_text = input.aimer.debug_small_gyro_detected ? "YES" : "NO";
      const std::string anti_top_text = input.aimer.debug_small_gyro_active ? "ON" : "OFF";
      tools::draw_text(
        vis,
        fmt::format(
          "  gyro: {}  anti-top: {}  id: {}", gyro_text, anti_top_text,
          input.aimer.debug_small_gyro_armor_id),
        {10, y0},
        input.aimer.debug_small_gyro_detected ? cv::Scalar(0, 200, 255)
                                              : cv::Scalar(200, 200, 255),
        0.6);
      y0 += line_h;

      const std::string fire_window_text =
        input.shooter.debug_small_gyro_fire_window ? "OPEN" : "CLOSE";
      if (input.shooter.debug_small_gyro_yaw_error_deg.has_value()) {
        tools::draw_text(
          vis,
          fmt::format(
            "  yaw_err: {:.2f} deg  fire_win: {}",
            input.shooter.debug_small_gyro_yaw_error_deg.value(), fire_window_text),
          {10, y0},
          input.shooter.debug_small_gyro_fire_window ? cv::Scalar(0, 255, 0)
                                                     : cv::Scalar(0, 165, 255),
          0.6);
      } else {
        tools::draw_text(vis, "  yaw_err: --  fire_win: --", {10, y0}, {160, 160, 160}, 0.6);
      }
      y0 += line_h;

      tools::draw_text(
        vis, fmt::format("  r:{:.2f} a:{:.1f}deg", x[8], x[6] * 57.3), {10, y0},
        {200, 200, 255}, 0.6);
      y0 += line_h;
    }
  } else {
    tools::draw_text(vis, "  (no target)", {10, y0}, {150, 150, 150}, 0.6);
    y0 += line_h;
  }

  // ----- 火控：control=aim 成功；shoot=control 且 Shooter 判定可开火 -----
  tools::draw_text(vis, "Fire control:", {10, y0}, {0, 255, 255}, 0.7);
  y0 += line_h;
  tools::draw_text(
    vis, fmt::format("  bullet_speed: {:.1f}  mode: {}", input.bullet_speed, io::MODES[input.mode]),
    {10, y0}, {255, 255, 255}, 0.6);
  y0 += line_h;
  tools::draw_text(
    vis,
    fmt::format("  control:{} shoot:{}", input.command.control ? 1 : 0, input.command.shoot ? 1 : 0),
    {10, y0}, {255, 255, 255}, 0.6);
  y0 += line_h;
  tools::draw_text(
    vis, fmt::format("  cmd yaw:{:.2f}  pitch:{:.2f}", input.command.yaw, input.command.pitch),
    {10, y0}, input.targets.empty() ? cv::Scalar(150, 150, 150) : cv::Scalar(255, 255, 255),
    0.6);
  if (input.targets.empty() && input.armors_before_tracker > 0) {
    tools::draw_text(
      vis, "  (no target -> fire control idle)", {10, y0 + line_h}, {0, 200, 255}, 0.55);
    y0 += line_h;
  }
  y0 += line_h;

  if (input.draw_armors.empty()) {
    tools::draw_text(
      vis, "no det: check enemy_color/min_confidence or aim at armor", {10, y0 + line_h},
      {0, 165, 255}, 0.55);
  }
}

void draw_target_debug(
  cv::Mat & vis, auto_aim::Solver & solver, const DebugOverlayInput & input)
{
  if (input.targets.empty()) return;

  const auto & target = input.targets.front();
  const Eigen::VectorXd x = target.ekf_x();

  if (x.size() >= 11) {
    draw_orbit(vis, solver, x, x[8], x[4], cv::Scalar(0, 255, 255));
    if (
      target.armor_xyza_list().size() == 4 &&
      (std::abs(x[9]) > 1e-3 || std::abs(x[10]) > 1e-3))
    {
      draw_orbit(vis, solver, x, x[8] + x[9], x[4] + x[10], cv::Scalar(0, 165, 255));
    }
  }

  // 四个装甲板（绿点，与 auto_aim_test 一致）、瞄准点（红点）。
  for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
    const auto image_points =
      solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
    tools::draw_points(vis, image_points, {0, 255, 0});
  }

  const auto aim_point = input.aimer.debug_aim_point;
  if (aim_point.valid) {
    const auto image_points = solver.reproject_armor(
      aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
    const bool fire_ready = input.command.shoot;
    const cv::Scalar fire_color = fire_ready ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 0, 255);
    tools::draw_points(vis, image_points, fire_color, fire_ready ? 4 : 2);
  }

  if (x.size() >= 5) {
    const std::vector<cv::Point3f> center_world = {
      cv::Point3f(static_cast<float>(x[0]), static_cast<float>(x[2]), static_cast<float>(x[4]))};
    const auto center_pixels = solver.world2pixel(center_world);
    if (!center_pixels.empty()) {
      const cv::Point center_point(cvRound(center_pixels[0].x), cvRound(center_pixels[0].y));
      if (
        center_point.x >= 0 && center_point.x < vis.cols && center_point.y >= 0 &&
        center_point.y < vis.rows)
      {
        cv::circle(vis, center_point, 8, cv::Scalar(255, 255, 0), 5, cv::LINE_AA);
        tools::draw_point(vis, center_point, {255, 255, 0}, 5);
      }
    }
  }
}
}  // namespace

bool show_debug_overlay(
  const cv::Mat & img, auto_aim::Solver & solver, const DebugOverlayInput & input)
{
  if (img.empty()) return true;

  // ===== 只显示下发给电控的 yaw / pitch =====
  // command.yaw / command.pitch 单位是「度」。
  // CBoard::send() 里会 ×10 编码成 int16 大端发给电控：
  //   data[0..1] = yaw×10,  data[2..3] = pitch×10
  // 所以这里的 raw 值就是电控实际收到的整数，排查时以它为准。
  cv::Mat vis = img.clone();

  // 检测框 + 目标调试（X 形等）—— 排查检测必须看这些
  draw_detection_boxes(vis, input.draw_armors);
  draw_target_debug(vis, solver, input);

  // 左下角信息面板（Tracker 状态 / EKF 估计 / 火控摘要）
  draw_text_panel(vis, input);

  const int yaw_raw = static_cast<int>(input.command.yaw * 10.0);
  const int pitch_raw = static_cast<int>(input.command.pitch * 10.0);

  tools::draw_text(
    vis, fmt::format("YAW   {:+9.2f} deg", input.command.yaw), {40, 70}, {0, 255, 255}, 1.5, 3);
  tools::draw_text(
    vis, fmt::format("PITCH {:+9.2f} deg", input.command.pitch), {40, 140}, {0, 255, 255}, 1.5, 3);
  tools::draw_text(
    vis, fmt::format("raw int16:  yaw {}   pitch {}", yaw_raw, pitch_raw), {40, 195},
    {200, 200, 200}, 0.8, 2);
  tools::draw_text(
    vis,
    fmt::format(
      "control {}   shoot {}", input.command.control ? 1 : 0, input.command.shoot ? 1 : 0),
    {40, 235}, {200, 200, 200}, 0.8, 2);


  cv::namedWindow("auto_aim", cv::WINDOW_NORMAL);
  cv::resizeWindow("auto_aim", 800, 600);
  cv::imshow("auto_aim", vis);
  return cv::waitKey(1) != 'q';
}

}  // namespace sentry_app
