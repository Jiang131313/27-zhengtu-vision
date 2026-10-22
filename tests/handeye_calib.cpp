// handeye_calib.cpp
// 手眼标定：棋盘格固定不动，手动转云台，记录多个位姿后自动解算 AX = XB。
//
// 用法：
//   ./build/handeye_calib --config-path=configs/demo.yaml --exposure-ms=30 --gain=15
//   空格 = 记录一个位姿；  回车/c = 解算并写入配置；  r = 重来；  q = 退出
//
// 原理（云台绕固定中心纯旋转，相机刚性固定在云台上）：
//   R_gripper2base / t_gripper2base : 云台姿态（电控 0x106 电机角度），原点在旋转中心 -> t 恒为 0
//   R_target2cam  / t_target2cam    : 棋盘格在相机系下的位姿（solvePnP）
//   -> cv::calibrateHandEye 解出 R_cam2gripper / t_cam2gripper
//      即项目里的 R_camera2gimbal / t_camera2gimbal
//
// 性能设计（重要）：
//   findChessboardCorners 很重（实测板子入画 200ms、不在画面里可达数秒），
//   如果和显示放在同一个线程里，检测期间画面会整个冻住。
//   因此这里把【检测】放到独立线程，主线程只负责取帧+显示，画面始终流畅。

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在 opencv2/core/eigen.hpp 之前
#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/cboard_motor_pose_buffer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

namespace
{
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml 配置文件路径 }"
  "{min-poses m    | 10    | 解算所需的最少位姿数 }"
  "{pattern-cols   | 6     | 棋盘格内角点列数 }"
  "{pattern-rows   | 6     | 棋盘格内角点行数 }"
  "{pattern-spacing-mm | 20 | 方格边长(mm) }"
  "{no-write       |       | 只解算不写入配置 }"
  "{exposure-ms e  | -1    | 覆盖曝光(ms)，-1=用配置文件里的（标定时建议 30）}"
  "{gain g         | -1    | 覆盖增益，-1=用配置文件里的（标定时建议 15）}"
  "{display-width  | 640   | 显示图宽度（越小 X11 越不堵）}"
  "{allow-subgrid  |       | 允许用子网格记录（默认禁止：手眼标定要求每帧都是同一块完整棋盘格）}";

std::string resolve_cli(
  const cv::CommandLineParser & cli, int argc, char * argv[], const std::string & long_name,
  const std::string & short_name)
{
  const std::string value = cli.get<std::string>(long_name);
  if (!value.empty() && value != "true" && value != "false") return value;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--" + long_name && i + 1 < argc) return argv[i + 1];
    if (arg == "-" + short_name && i + 1 < argc) return argv[i + 1];
    const std::string p1 = "--" + long_name + "=";
    if (arg.rfind(p1, 0) == 0) return arg.substr(p1.size());
    const std::string p2 = "-" + short_name + "=";
    if (arg.rfind(p2, 0) == 0) return arg.substr(p2.size());
  }
  return value;
}

// scale<1 时在缩小图上找角点（findChessboardCorners 很重），再把角点还原到原图做亚像素精化，
// 精度不受降采样影响。
bool detect_chessboard(
  const cv::Mat & frame, const cv::Size & pattern, std::vector<cv::Point2f> & corners,
  double scale)
{
  corners.clear();
  cv::Mat src;
  if (scale < 1.0) {
    cv::resize(frame, src, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    src = frame;
  }

  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  // 不加 CALIB_CB_FAST_CHECK：真实场景下它会把有效棋盘格误判为 NO
  std::vector<cv::Point2f> found_corners;
  bool found = cv::findChessboardCorners(
    gray, pattern, found_corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!found) {
    // 经典检测器对外圈方格对比度低/轻微模糊/光照不均很敏感，常见症状是
    // "整块 NxN 找不到，只认出中间子网格"。findChessboardCornersSB 是基于
    // 分块的检测器，在这些情况下明显更稳，作为兜底再试一次。
    found = cv::findChessboardCornersSB(
      gray, pattern, found_corners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    if (found) {
      static bool sb_notified = false;
      if (!sb_notified) {
        sb_notified = true;
        tools::logger()->info(
          "[detect] 经典检测器失败，改用 findChessboardCornersSB 成功（{}x{}）",
          pattern.width, pattern.height);
      }
    }
  }
  if (!found) return false;

  corners.reserve(found_corners.size());
  for (const auto & c : found_corners) {
    corners.emplace_back(static_cast<float>(c.x / scale), static_cast<float>(c.y / scale));
  }

  cv::Mat gray_full;
  cv::cvtColor(frame, gray_full, cv::COLOR_BGR2GRAY);
  cv::cornerSubPix(
    gray_full, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
  return true;
}

cv::Mat to_T(const cv::Mat & R, const cv::Mat & t)
{
  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t.reshape(1, 3).copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}

// 检测结果（检测线程 -> 主线程）
struct Shared
{
  std::mutex mtx;
  cv::Mat frame;                                  // 最新帧（主线程写入）
  std::chrono::steady_clock::time_point frame_t;  // 该帧的时间戳
  bool frame_new = false;

  bool found = false;                    // 检测结果
  std::vector<cv::Point2f> corners;
  cv::Size size_used{0, 0};              // 实际检测成功的内角点尺寸
  double scale_used = 1.0;
  double last_detect_ms = 0.0;
  std::chrono::steady_clock::time_point detect_done_t;

  // 检测成功那一刻的云台姿态（与检测用的那帧严格对应）
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  bool q_valid = false;
  std::chrono::steady_clock::time_point q_t;
};
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const std::string config_path = resolve_cli(cli, argc, argv, "config-path", "c");
  const int min_poses = cli.get<int>("min-poses");
  const int pat_cols = cli.get<int>("pattern-cols");
  const int pat_rows = cli.get<int>("pattern-rows");
  const double spacing_mm = cli.get<double>("pattern-spacing-mm");
  const bool write_back = !cli.has("no-write");
  const int display_width = cli.get<int>("display-width");
  const bool allow_subgrid = cli.has("allow-subgrid");
  const cv::Size pattern_size(pat_cols, pat_rows);

  tools::Exiter exiter;


  // 先把窗口建出来并显示"等待相机"：海康 MVS SDK 从启动到出第一帧要数秒
  {
    cv::Mat splash(360, display_width, CV_8UC3, cv::Scalar(32, 32, 32));
    tools::draw_text(splash, "handeye calibration", {30, 150}, {255, 255, 255}, 0.6, 2);
    tools::draw_text(splash, "waiting for camera ...", {30, 190}, {0, 255, 255}, 0.7, 2);
    cv::imshow("handeye_calib", splash);
    cv::waitKey(50);
    tools::logger()->info("[handeye] 窗口已弹出（正在等待相机...）");
  }

  auto yaml = YAML::LoadFile(config_path);

  // ---- 相机内参 ----
  const auto cm_data = yaml["camera_matrix"].as<std::vector<double>>();
  const auto dc_data = yaml["distort_coeffs"].as<std::vector<double>>();
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) K.at<double>(i / 3, i % 3) = cm_data[i];
  cv::Mat D = cv::Mat::zeros(1, static_cast<int>(dc_data.size()), CV_64F);
  for (size_t i = 0; i < dc_data.size(); ++i) D.at<double>(0, static_cast<int>(i)) = dc_data[i];

  const double offset_s = yaml["imu_time_offset_s"] ? yaml["imu_time_offset_s"].as<double>() : 0.0;
  const auto imu_time_offset =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(offset_s));

  // 可选：覆盖曝光/增益（demo.yaml 的曝光通常偏暗）
  std::string camera_config_path = config_path;
  const double cli_exposure_ms = cli.get<double>("exposure-ms");
  const double cli_gain = cli.get<double>("gain");
  if (cli_exposure_ms > 0 || cli_gain >= 0) {
    std::ifstream in(config_path);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    const auto set_key = [&](std::string text, const std::string & key, const std::string & val) {
      const std::string k = key + ":";
      const size_t pos = text.find(k);
      if (pos == std::string::npos) return text;
      const size_t eol = text.find('\n', pos);
      return text.substr(0, pos) + k + " " + val +
             (eol == std::string::npos ? "" : text.substr(eol));
    };
    if (cli_exposure_ms > 0) content = set_key(content, "exposure_ms", fmt::format("{:.1f}", cli_exposure_ms));
    if (cli_gain >= 0) content = set_key(content, "gain", fmt::format("{:.1f}", cli_gain));
    camera_config_path = "/tmp/handeye_cam_override.yaml";
    std::ofstream fo(camera_config_path);
    fo << content;
    fo.close();
    tools::logger()->info(
      "曝光/增益已覆盖: exposure_ms={:.1f}, gain={:.1f}", cli_exposure_ms, cli_gain);
  }

  io::CBoard cboard(config_path);
  io::CBoardMotorPoseBuffer motor_pose;
  cboard.set_motor_pose_buffer(&motor_pose);
  io::Camera camera(camera_config_path);
  auto_aim::Solver solver(config_path);

  // 棋盘格三维点
  std::vector<cv::Point3f> object_points;
  object_points.reserve(pattern_size.area());
  for (int r = 0; r < pat_rows; ++r) {
    for (int c = 0; c < pat_cols; ++c) {
      object_points.emplace_back(
        static_cast<float>(c * spacing_mm), static_cast<float>(r * spacing_mm), 0.0F);
    }
  }

  Shared shared;
  std::atomic<bool> stop{false};

  // ---------- 检测线程 ----------
  std::thread detect_thread([&]() {
    int miss_streak = 0;
    while (!stop.load()) {
      cv::Mat frame;
      std::chrono::steady_clock::time_point ft;
      {
        std::lock_guard<std::mutex> lk(shared.mtx);
        if (!shared.frame_new || shared.frame.empty()) {
          // 没有新帧，睡一会儿
        } else {
          frame = shared.frame.clone();
          ft = shared.frame_t;
          shared.frame_new = false;
        }
      }
      if (frame.empty()) {
        std::this_thread::sleep_for(5ms);
        continue;
      }

      // 连续找不到时退避：只跑最快那一档，避免每轮白跑 3 遍（数秒）
      // 优先用最高分辨率：角点定位精度直接决定 solvePnP 算出的板子姿态精度。
      // 原来先试 0.4（最快），成功了就不试精细档，导致板子稍远时每格只有十几像素、
      // 每帧姿态差好几度 —— 手眼方程直接解不通（实测旋转角比值在 0.55~1.61 乱跳）。
      const std::vector<double> scales = (miss_streak >= 2)
        ? std::vector<double>{0.4}
        : std::vector<double>{1.0, 0.7, 0.4};
      const auto t0 = std::chrono::steady_clock::now();
      bool found = false;
      std::vector<cv::Point2f> corners;
      double used = 1.0;
      cv::Size used_size = pattern_size;
      // 先在配置尺寸下试各种缩放；找不到就逐级降尺寸 ——
      // 太暗/没白边时外圈角点会消失，但内层子网格依旧是合法标定目标（间距不变）。
      for (int sz = pattern_size.width; sz >= 4 && !found; --sz) {
        const cv::Size ps(sz, sz);
        for (const double sc : scales) {
          if (detect_chessboard(frame, ps, corners, sc)) {
            found = true;
            used = sc;
            used_size = ps;
            break;
          }
        }
      }
      const auto t1 = std::chrono::steady_clock::now();
      miss_streak = found ? 0 : miss_streak + 1;

      // 一次性诊断：启动 8 秒后仍检测不到，就把 4x4~10x10 都试一遍，报出实际可用的尺寸
      static bool probed = false;
      static const auto t_probe_after = std::chrono::steady_clock::now() + 8s;
      // 注意：只在"完全找不到"时探测是死代码——大多数情况能认出子网格(found=true)。
      // 所以"只认出比配置小的子网格"时也要探测一次，才能真正报出可用尺寸。
      const bool subgrid_only = found && used_size != pattern_size;
      if (!probed && (subgrid_only || !found) && std::chrono::steady_clock::now() > t_probe_after) {
        probed = true;
        tools::logger()->warn("[probe] 一直检测不到，开始全尺寸探测（4x4~10x10）...");
        int hits = 0;
        for (int sz = 4; sz <= 10; ++sz) {
          std::vector<cv::Point2f> c;
          if (detect_chessboard(frame, cv::Size(sz, sz), c, 1.0)) {
            tools::logger()->warn(
              "[probe] ✅ {}x{} 内角点可检测到（当前配置 {}x{}）", sz, sz, pat_cols, pat_rows);
            ++hits;
          }
        }
        if (hits == 0) {
          tools::logger()->warn(
            "[probe] ❌ 4x4~10x10 全部失败 → 不是尺寸问题，检查：\n"
            "        ① 外圈白边（棋盘格外围要有至少一格宽的白边）\n"
            "        ② 板子平整（纸弯了检测不到）\n"
            "        ③ 光照（太暗/反光/阴影）\n"
            "        ④ 板子是否完整在画面内");
        } else {
          tools::logger()->warn("[probe] 请把 configs/calibration.yaml 的 pattern_cols/rows 改成上面报出的尺寸");
        }
      }

      // 该帧对应的云台姿态（无论是否检测成功都取，供"当前姿态"显示）
      const auto q = motor_pose.imu_at(ft + imu_time_offset);
      const auto q_t = ft;

      {
        std::lock_guard<std::mutex> lk(shared.mtx);
        shared.found = found;
        shared.corners = corners;
        shared.size_used = used_size;
        shared.scale_used = used;
        shared.last_detect_ms = std::chrono::duration<double>(t1 - t0).count() * 1000.0;
        shared.detect_done_t = t1;
        shared.q = q;
        shared.q_t = q_t;
        shared.q_valid = cboard.has_motor_pose();
      }
    }
  });

  std::vector<cv::Mat> R_g2b, t_g2b, R_t2c, t_t2c;
  auto last_capture = std::chrono::steady_clock::now() - 10s;
  bool solved = false;
  cv::Mat R_cam2gripper, t_cam2gripper;
  double resid_t_mm = 0.0;

  tools::logger()->info(
    "[handeye] 棋盘格 {}x{}  间距 {:.1f}mm  最少 {} 个位姿  配置: {}", pat_cols, pat_rows,
    spacing_mm, min_poses, config_path);
  tools::logger()->info("[handeye] 棋盘格固定 -> 手动转云台 -> 空格记录 -> 回车解算");
  tools::logger()->info(
    "[handeye] 姿态来源: {}",
    cboard.has_motor_pose() ? "电控电机角度(0x106)" : "不可用（无法记录）");

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;

    // 把最新帧交给检测线程（不阻塞）
    {
      std::lock_guard<std::mutex> lk(shared.mtx);
      shared.frame = img;
      shared.frame_t = t;
      shared.frame_new = true;
    }

    // 读取最近一次检测结果（不阻塞）
    bool found = false;
    std::vector<cv::Point2f> corners;
    double det_ms = 0.0;
    double scale_used = 1.0;
    cv::Size size_used(pat_cols, pat_rows);
    Eigen::Quaterniond q_now = Eigen::Quaterniond::Identity();
    bool q_valid = false;
    std::chrono::steady_clock::time_point det_done = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lk(shared.mtx);
      found = shared.found;
      corners = shared.corners;
      det_ms = shared.last_detect_ms;
      scale_used = shared.scale_used;
      if (shared.size_used.width > 0) size_used = shared.size_used;
      q_now = shared.q;
      q_valid = shared.q_valid;
      det_done = shared.detect_done_t;
    }

    // 检测是异步的：结果对应的是"若干毫秒~数秒前"的那一帧。
    // 结果太旧就标注出来，并且不画角点 —— 否则看起来像"明明在中间却说没检测到"。
    const double result_age =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - det_done).count();
    const bool result_fresh = result_age < 0.8;

    // ---- 显示：缩小后再 imshow（X11 传原始像素很费带宽）----
    const double disp_scale = static_cast<double>(display_width) / static_cast<double>(img.cols);
    cv::Mat display;
    cv::resize(img, display, cv::Size(), disp_scale, disp_scale, cv::INTER_AREA);

    if (found && result_fresh) {
      std::vector<cv::Point2f> sc;
      sc.reserve(corners.size());
      for (const auto & c : corners) {
        sc.emplace_back(static_cast<float>(c.x * disp_scale), static_cast<float>(c.y * disp_scale));
      }
      cv::drawChessboardCorners(display, size_used, sc, found);
    }

    const bool flash =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_capture).count() < 0.6;
    if (flash) {
      cv::rectangle(
        display, cv::Rect(0, 0, display.cols - 1, display.rows - 1), cv::Scalar(0, 255, 0), 6);
    }

    int y = 22;
    const int dy = 22;
    tools::draw_text(
      display, fmt::format("board {}x{} spacing {:.1f}mm", pat_cols, pat_rows, spacing_mm), {10, y},
      {255, 255, 255}, 0.5, 1);
    y += dy;
    if (result_fresh) {
      tools::draw_text(
        display,
        fmt::format(
          "found: {}  {}  poses: {}/{}", found ? "YES" : "NO",
          found ? fmt::format("({}x{} @{:.1f})", size_used.width, size_used.height, scale_used)
                : "",
          R_g2b.size(), min_poses),
        {10, y}, found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 0.5, 1);
    } else {
      // 结果还没出来/已过期：不要显示成"没检测到"，那会误导
      tools::draw_text(
        display,
        fmt::format(
          "检测中... (上次结果 {:.1f}s 前)  poses: {}/{}", result_age, R_g2b.size(), min_poses),
        {10, y}, cv::Scalar(160, 160, 160), 0.5, 1);
    }
    y += dy;
    if (q_valid) {
      const Eigen::Vector3d ypr = tools::eulers(q_now.toRotationMatrix(), 2, 1, 0) * 180.0 / M_PI;
      tools::draw_text(
        display, fmt::format("gimbal yaw {:.1f}  pitch {:.1f}", ypr[0], ypr[1]), {10, y},
        {0, 255, 255}, 0.5, 1);
      y += dy;
    }
    tools::draw_text(
      display, fmt::format("detect {:.0f} ms", det_ms), {10, y}, {180, 180, 180}, 0.45, 1);
    y += dy;
    if (flash) {
      tools::draw_text(
        display, fmt::format("POSED {}", R_g2b.size()), {display.cols / 2 - 70, display.rows / 2},
        {0, 255, 0}, 1.0, 2);
    }
    if (solved) {
      tools::draw_text(
        display,
        fmt::format(
          "t = [{:.3f},{:.3f},{:.3f}] m   resid {:.1f}mm", t_cam2gripper.at<double>(0),
          t_cam2gripper.at<double>(1), t_cam2gripper.at<double>(2), resid_t_mm),
        {10, y}, {0, 255, 255}, 0.45, 1);
      y += dy;
    }
    tools::draw_text(
      display, "SPACE=record  ENTER=solve+write  r=reset  q=quit", {10, y}, {200, 200, 200}, 0.45, 1);

    // 注意：不要用 namedWindow/resizeWindow/moveWindow，X11 转发下会让窗口渲染失效
    cv::imshow("handeye_calib", display);
    const int key = cv::waitKey(1);
    if (key == 'q') break;

    if (key == ' ') {
      // 用最近一次成功的检测结果 + 它对应的云台姿态（两者严格对应同一帧）
      Shared snap;
      {
        std::lock_guard<std::mutex> lk(shared.mtx);
        const double age = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - shared.detect_done_t)
                             .count();
        if (!shared.found || shared.corners.empty()) {
          tools::logger()->warn("当前没有有效的棋盘格检测，忽略");
          continue;
        }
        if (age > 1.0) {
          tools::logger()->warn("检测结果已过期({:.1f}s 前)，等它刷新后再按空格", age);
          continue;
        }
        // 手眼标定要求每帧都是【同一块完整棋盘格】。
        // 如果只检测到子网格（例如配置 7x7 却只找到 5x5），各帧的目标坐标系原点
        // 在板子上位置不同 -> 目标"乱跳" -> AX=XB 方程失效 -> 结果必错。
        if (!allow_subgrid && shared.size_used.width > 0 &&
            (shared.size_used.width != pat_cols || shared.size_used.height != pat_rows)) {
          tools::logger()->warn(
            "❌ 拒绝记录：配置的是 {}x{}，实际只检测到 {}x{}（子网格）。\n"
            "   手眼标定必须用完整棋盘格 —— 请改善光照/加白边，让整块板子被稳定检测到；\n"
            "   或者如果板子实际就是 {}x{} 内角点，改配置 --pattern-cols={} --pattern-rows={}。\n"
            "   （确实要用子网格请加 --allow-subgrid，但结果不可信）",
            pat_cols, pat_rows, shared.size_used.width, shared.size_used.height,
            shared.size_used.width, shared.size_used.height, shared.size_used.width,
            shared.size_used.height);
          continue;
        }
        if (!shared.q_valid) {
          tools::logger()->warn("没有可用的云台姿态（检查电控是否发 0x106）");
          continue;
        }
        // 按"实际检测到的尺寸"生成三维点（可能比配置小，间距仍是 spacing_mm）
        std::vector<cv::Point3f> obj;
        const cv::Size us = shared.size_used.width > 0 ? shared.size_used : pattern_size;
        obj.reserve(us.area());
        for (int r = 0; r < us.height; ++r) {
          for (int c = 0; c < us.width; ++c) {
            obj.emplace_back(
              static_cast<float>(c * spacing_mm), static_cast<float>(r * spacing_mm), 0.0F);
          }
        }
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(obj, shared.corners, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
          tools::logger()->warn("solvePnP 失败，忽略");
          continue;
        }
        cv::Mat R_t;
        cv::Rodrigues(rvec, R_t);
        cv::Mat R_g = cv::Mat::zeros(3, 3, CV_64F);
        const Eigen::Matrix3d Rm = shared.q.toRotationMatrix();
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c) R_g.at<double>(r, c) = Rm(r, c);

        R_g2b.push_back(R_g);
        t_g2b.push_back(cv::Mat::zeros(3, 1, CV_64F));  // 原点在旋转中心 -> 纯旋转
        R_t2c.push_back(R_t);
        t_t2c.push_back(tvec);

        // 诊断：把每一帧的原始数据落盘，便于离线穷举运动学模型
        // 每行 21 个数：R_g2b(9) R_t2c(9) t_t2c(3)
        {
          static std::ofstream pose_dump("/tmp/handeye_poses.txt", std::ios::trunc);
          for (int rr = 0; rr < 3; ++rr)
            for (int cc = 0; cc < 3; ++cc) pose_dump << R_g.at<double>(rr, cc) << " ";
          for (int rr = 0; rr < 3; ++rr)
            for (int cc = 0; cc < 3; ++cc) pose_dump << R_t.at<double>(rr, cc) << " ";
          for (int rr = 0; rr < 3; ++rr) pose_dump << tvec.at<double>(rr) << " ";
          pose_dump << "\n";
          pose_dump.flush();
        }

        const Eigen::Vector3d ypr = tools::eulers(Rm, 2, 1, 0) * 180.0 / M_PI;
        tools::logger()->info(
          "✅ 记录位姿 {}/{}   ({}x{})   云台 yaw={:.1f}° pitch={:.1f}°", R_g2b.size(), min_poses,
          us.width, us.height, ypr[0], ypr[1]);
        last_capture = std::chrono::steady_clock::now();
      }
      continue;
    }

    if (key == 'r') {
      R_g2b.clear();
      t_g2b.clear();
      R_t2c.clear();
      t_t2c.clear();
      solved = false;
      tools::logger()->info("已清空，重新记录");
      continue;
    }

    if (key == '\r' || key == '\n' || key == 'c') {
      if (static_cast<int>(R_g2b.size()) < min_poses) {
        tools::logger()->warn("位姿太少：{} 个，至少需要 {} 个", R_g2b.size(), min_poses);
        continue;
      }
      cv::calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);

      const cv::Mat X = to_T(R_cam2gripper, t_cam2gripper);
      const cv::Mat T_ref = to_T(R_g2b[0], t_g2b[0]) * X * to_T(R_t2c[0], t_t2c[0]);
      double max_t = 0.0;
      double sum_t = 0.0;
      for (size_t i = 0; i < R_g2b.size(); ++i) {
        const cv::Mat pred = to_T(R_g2b[i], t_g2b[i]) * X * to_T(R_t2c[i], t_t2c[i]);
        const cv::Mat d = pred(cv::Rect(3, 0, 1, 3)) - T_ref(cv::Rect(3, 0, 1, 3));
        const double e = cv::norm(d) * 1000.0;
        max_t = std::max(max_t, e);
        sum_t += e;
      }
      const double mean_t = sum_t / static_cast<double>(R_g2b.size());
      const double max_t_global = max_t;
      resid_t_mm = mean_t;

      // ---- ① 旋转多样性检查：云台没怎么转，标定必然不准 ----
      double max_ang_deg = 0.0;
      for (size_t i = 0; i < R_g2b.size(); ++i) {
        for (size_t j = i + 1; j < R_g2b.size(); ++j) {
          const cv::Mat dR = R_g2b[i].t() * R_g2b[j];
          const double tr =
            dR.at<double>(0, 0) + dR.at<double>(1, 1) + dR.at<double>(2, 2);
          const double c = std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0));
          max_ang_deg = std::max(max_ang_deg, std::acos(c) * 180.0 / M_PI);
        }
      }

      // ---- ② 合理性检查：相机离云台旋转中心不可能超过 0.5 米 ----
      const double t_norm = cv::norm(t_cam2gripper);
      const bool plausible = (t_norm < 0.5) && (max_t_global < 100.0);

      solved = plausible;

      tools::logger()->info("========== 手眼标定结果 ==========");
      tools::logger()->info(
        "R_camera2gimbal = [{:.10f}, {:.10f}, {:.10f}, {:.10f}, {:.10f}, {:.10f}, {:.10f}, "
        "{:.10f}, {:.10f}]",
        R_cam2gripper.at<double>(0, 0), R_cam2gripper.at<double>(0, 1), R_cam2gripper.at<double>(0, 2),
        R_cam2gripper.at<double>(1, 0), R_cam2gripper.at<double>(1, 1), R_cam2gripper.at<double>(1, 2),
        R_cam2gripper.at<double>(2, 0), R_cam2gripper.at<double>(2, 1), R_cam2gripper.at<double>(2, 2));
      tools::logger()->info(
        "t_camera2gimbal = [{:.10f}, {:.10f}, {:.10f}]   (m, 相机在云台系下的位置)",
        t_cam2gripper.at<double>(0), t_cam2gripper.at<double>(1), t_cam2gripper.at<double>(2));
      tools::logger()->info(
        "一致性残差: 平均 {:.2f} mm, 最大 {:.2f} mm  (>20mm 建议重标)", mean_t, max_t);
      tools::logger()->info("云台转动范围: 最大夹角 {:.1f}° (>30° 较好)", max_ang_deg);
      if (max_ang_deg < 15.0) {
        tools::logger()->warn("⚠️ 云台转动量太小（{:.1f}°），标定会不准，建议多转一些角度重标", max_ang_deg);
      }

      if (!plausible) {
        tools::logger()->error(
          "❌ 结果不合理，【不予写入配置】：|t|={:.3f} m（相机不可能离云台这么远），"
          "最大残差 {:.0f} mm",
          t_norm, max_t_global);
        tools::logger()->error(
          "最常见原因：棋盘格在各帧被检测到【不同位置】，目标坐标系乱跳。"
          "手眼标定必须保证每一帧检测到的是【同一块完整棋盘格】——"
          "不要依赖子网格（例如板子 7x7 却只检测到 5x5）。");
        tools::logger()->error("请先改善光照/加白边，让整块板子被稳定检测到，再重标。");
        continue;
      }

      if (write_back) {
        std::ifstream in(config_path);
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();
        // 备份：已存在就不覆盖（避免第二次运行把好的备份冲掉）
        const std::string bak_path = config_path + ".bak_handeye";
        {
          std::ifstream probe(bak_path);
          if (!probe.good()) {
            std::ofstream bak(bak_path);
            bak << content;
            bak.close();
            tools::logger()->info("已备份原配置到 {}", bak_path);
          } else {
            tools::logger()->info("备份 {} 已存在，不覆盖", bak_path);
          }
        }
        const std::string new_R = fmt::format(
          "R_camera2gimbal: [{:.10f}, {:.10f}, {:.10f}, {:.10f}, {:.10f}, {:.10f}, {:.10f}, "
          "{:.10f}, {:.10f}]",
          R_cam2gripper.at<double>(0, 0), R_cam2gripper.at<double>(0, 1),
          R_cam2gripper.at<double>(0, 2), R_cam2gripper.at<double>(1, 0),
          R_cam2gripper.at<double>(1, 1), R_cam2gripper.at<double>(1, 2),
          R_cam2gripper.at<double>(2, 0), R_cam2gripper.at<double>(2, 1),
          R_cam2gripper.at<double>(2, 2));
        const std::string new_t = fmt::format(
          "t_camera2gimbal: [{:.10f}, {:.10f}, {:.10f}]", t_cam2gripper.at<double>(0),
          t_cam2gripper.at<double>(1), t_cam2gripper.at<double>(2));
        const auto replace_key = [&](std::string text, const std::string & key,
                                     const std::string & replacement) {
          const std::string k = key + ":";
          const size_t pos = text.find(k);
          if (pos == std::string::npos) return text;
          const size_t eol = text.find('\n', pos);
          return text.substr(0, pos) + replacement +
                 (eol == std::string::npos ? "" : text.substr(eol));
        };
        std::string out = replace_key(content, "R_camera2gimbal", new_R);
        out = replace_key(out, "t_camera2gimbal", new_t);
        std::ofstream fo(config_path);
        fo << out;
        fo.close();
        tools::logger()->info("已写入 {}（备份 {}）", config_path, config_path + ".bak_handeye");
      } else {
        tools::logger()->info("(--no-write: 未写入配置)");
      }
      continue;
    }
  }

  stop.store(true);
  if (detect_thread.joinable()) detect_thread.join();
  return 0;
}
