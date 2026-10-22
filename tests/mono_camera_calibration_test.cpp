#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明 }"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径   }"
  "{output o       | assets/mono_camera_calibration.yaml | 输出标定结果路径 }"
  "{pattern p      | circles                  | 标定板类型：circles/chessboard }"
  "{min-samples m  | 12                       | 允许标定的最少样本数 }"
  "{auto-target n  | 20                       | 全自动：采够该样本数自动标定+写出+退出（0=关闭）}";

enum class PatternType
{
  circles,
  chessboard,
};

PatternType parse_pattern_type(const std::string & pattern)
{
  if (pattern == "circles") return PatternType::circles;
  if (pattern == "chessboard") return PatternType::chessboard;
  throw std::runtime_error("pattern must be circles or chessboard");
}

std::string resolve_cli_value(
  const cv::CommandLineParser & cli, int argc, char * argv[], const std::string & long_name,
  std::string_view short_name)
{
  const std::string value = cli.get<std::string>(long_name);
  if (!value.empty() && value != "true" && value != "false") return value;

  const std::string short_opt = fmt::format("-{}", short_name);
  const std::string long_opt = fmt::format("--{}", long_name);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == short_opt || arg == long_opt) && i + 1 < argc) {
      return argv[i + 1];
    }
    const std::string long_prefix = long_opt + "=";
    if (arg.rfind(long_prefix, 0) == 0) {
      return arg.substr(long_prefix.size());
    }
    const std::string short_prefix = short_opt + "=";
    if (arg.rfind(short_prefix, 0) == 0) {
      return arg.substr(short_prefix.size());
    }
  }

  return value;
}

std::vector<cv::Point3f> build_object_points(const cv::Size & pattern_size, float spacing_mm)
{
  std::vector<cv::Point3f> object_points;
  object_points.reserve(pattern_size.area());
  for (int row = 0; row < pattern_size.height; ++row) {
    for (int col = 0; col < pattern_size.width; ++col) {
      object_points.emplace_back(col * spacing_mm, row * spacing_mm, 0.0f);
    }
  }
  return object_points;
}

bool detect_pattern(
  const cv::Mat & frame, PatternType pattern_type, const cv::Size & pattern_size,
  std::vector<cv::Point2f> & corners)
{
  corners.clear();
  if (pattern_type == PatternType::circles) {
    return cv::findCirclesGrid(frame, pattern_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
  }

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  // 注意：不要加 cv::CALIB_CB_FAST_CHECK —— 它是激进的前置筛查，
  // 真实场景（光照不均/轻微模糊/倾斜）下会把有效棋盘格判成 NO。
  //
  // 性能：棋盘格检测的代价与像素数成正比。在整幅 1440x1080 上直接检测会占满主线程
  // （该线程同时负责 imshow/waitKey），窗口会被系统判定“无响应”。
  // 改为在半分辨率上检测（快约 4 倍，方格仍有 ~30px，足够），
  // 再把角点乘回全分辨率，最后在全分辨率上做 cornerSubPix 精修。
  static constexpr double kDetectScale = 0.5;
  cv::Mat small;
  cv::resize(gray, small, cv::Size(), kDetectScale, kDetectScale, cv::INTER_AREA);

  bool found = cv::findChessboardCorners(
    small, pattern_size, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!found) return false;

  for (auto & p : corners) p *= static_cast<float>(1.0 / kDetectScale);

  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
  return true;
}

double compute_mean_corner_shift(
  const std::vector<cv::Point2f> & lhs, const std::vector<cv::Point2f> & rhs)
{
  if (lhs.size() != rhs.size() || lhs.empty()) return std::numeric_limits<double>::infinity();

  double sum = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    sum += cv::norm(lhs[i] - rhs[i]);
  }
  return sum / static_cast<double>(lhs.size());
}

double compute_reprojection_error(
  const std::vector<std::vector<cv::Point3f>> & object_points,
  const std::vector<std::vector<cv::Point2f>> & image_points, const std::vector<cv::Mat> & rvecs,
  const std::vector<cv::Mat> & tvecs, const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs)
{
  double total_error = 0.0;
  size_t total_points = 0;
  for (size_t i = 0; i < object_points.size(); ++i) {
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(
      object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, reprojected);
    for (size_t j = 0; j < reprojected.size(); ++j) {
      total_error += cv::norm(image_points[i][j] - reprojected[j]);
    }
    total_points += reprojected.size();
  }
  return total_points == 0 ? 0.0 : total_error / static_cast<double>(total_points);
}

void write_calibration_yaml(
  const std::string & output_path, const cv::Mat & camera_matrix, const cv::Mat & dist_coeffs,
  double reprojection_error, const cv::Size & image_size)
{
  std::filesystem::path path(output_path);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }

  YAML::Emitter out;
  std::vector<double> camera_matrix_data(camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> dist_coeffs_data(dist_coeffs.begin<double>(), dist_coeffs.end<double>());

  out << YAML::BeginMap;
  out << YAML::Comment(fmt::format("重投影误差: {:.4f}px", reprojection_error));
  out << YAML::Key << "image_width" << YAML::Value << image_size.width;
  out << YAML::Key << "image_height" << YAML::Value << image_size.height;
  out << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << camera_matrix_data;
  out << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow << dist_coeffs_data;
  out << YAML::EndMap;

  std::ofstream fout(output_path);
  fout << out.c_str() << "\n";
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const std::string config_path = resolve_cli_value(cli, argc, argv, "config-path", "c");
  const std::string output_path = resolve_cli_value(cli, argc, argv, "output", "o");
  const int min_samples = cli.get<int>("min-samples");
  const int auto_target = cli.get<int>("auto-target");
  const PatternType pattern_type = parse_pattern_type(resolve_cli_value(cli, argc, argv, "pattern", "p"));

  YAML::Node yaml = tools::load(config_path);
  const int pattern_cols = tools::read<int>(yaml, "pattern_cols");
  const int pattern_rows = tools::read<int>(yaml, "pattern_rows");
  const float spacing_mm = static_cast<float>(tools::read<double>(yaml, "center_distance_mm"));
  const cv::Size pattern_size(pattern_cols, pattern_rows);
  const auto object_template = build_object_points(pattern_size, spacing_mm);

  io::Camera camera(config_path);

  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;

  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<cv::Point2f> corners;
  std::vector<cv::Point2f> last_auto_sample;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double reprojection_error = 0.0;
  cv::Size image_size;

  // 实际使用的内角点尺寸：配置尺寸找不到时自动降级到能用的最大尺寸
  cv::Size used_size = pattern_size;
  bool auto_size_warned = false;

  bool calibrated = false;
  bool show_undistort = false;
  bool auto_collect = true;  // 默认开启：拿着板子转就能自动采集
  auto last_auto_collect_time = std::chrono::steady_clock::now() - 2s;
  // 采集成功提示：记录最近一次成功的时刻，窗口上会闪绿框+大字
  auto last_capture_time = std::chrono::steady_clock::now() - 10s;

  // 标定（返回是否成功）
  auto do_calibrate = [&]() -> bool {
    if (static_cast<int>(image_points.size()) < min_samples) {
      tools::logger()->warn(
        "Need at least {} samples before calibration, current {}", min_samples,
        image_points.size());
      return false;
    }
    std::vector<cv::Mat> rvecs, tvecs;
    auto criteria =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
    double rms = cv::calibrateCamera(
      object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs,
      cv::CALIB_FIX_K3, criteria);
    reprojection_error =
      compute_reprojection_error(object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs);
    calibrated = true;
    tools::logger()->info(
      "Calibration done. rms {:.4f}, reprojection error {:.4f}px", rms, reprojection_error);
    return true;
  };

  // 写 yaml
  auto do_write = [&]() {
    if (!calibrated) {
      tools::logger()->warn("Calibrate first, then write yaml");
      return;
    }
    write_calibration_yaml(output_path, camera_matrix, dist_coeffs, reprojection_error, image_size);
    tools::logger()->info("Calibration yaml written to {}", output_path);
  };

  if (auto_target > 0 && auto_collect) {
    tools::logger()->info(
      "全自动模式：自动采集 {} 个样本后自动标定并写出（按 q 可随时中止）", auto_target);
  }
  tools::logger()->info("Press s to save a sample, c to calibrate, w to write yaml");
  tools::logger()->info("Press a to toggle auto collect, u to toggle undistort preview, r to reset");

  while (true) {
    camera.read(frame, timestamp);
    if (frame.empty()) continue;
    image_size = frame.size();

    // 先把上次成功的尺寸放在最前面试：一旦确定板子实际格数，
    // 每帧就只需一次检测（否则每帧要试多个尺寸，把主线程占满）。
    bool found = detect_pattern(frame, pattern_type, used_size, corners);

    // 配置尺寸找不到时，自动降级到能用的最大尺寸（外圈被挡/板子不平时很有用）。
    // 子网格同样是合法标定目标：方格间距不变，结果一样正确，只是用的点少一些。
    if (!found) {
      for (int s = pattern_cols - 1; s >= 4; --s) {
        if (s == used_size.width) continue;
        const cv::Size try_size(s, s);
        if (detect_pattern(frame, pattern_type, try_size, corners)) {
          used_size = try_size;
          found = true;
          if (!auto_size_warned) {
            auto_size_warned = true;
            tools::logger()->warn(
              "[auto] 配置的 {}x{} 找不到，自动改用 {}x{}（板子外圈可能被挡或不平）",
              pattern_cols, pattern_rows, s, s);
          }
          break;
        }
      }
    }

    if (
      auto_collect && found &&
      std::chrono::steady_clock::now() - last_auto_collect_time > 800ms &&
      compute_mean_corner_shift(corners, last_auto_sample) > 12.0)
    {
      image_points.push_back(corners);
      object_points.push_back(build_object_points(used_size, spacing_mm));
      last_auto_sample = corners;
      last_auto_collect_time = std::chrono::steady_clock::now();
      last_capture_time = std::chrono::steady_clock::now();
      tools::logger()->info(
        "✅ 采集成功 {}/{}  ({}x{}) —— 移动板子继续采集", image_points.size(), auto_target,
        used_size.width, used_size.height);
    }

    // 全自动：采够 auto_target 个样本 → 标定 → 写 yaml → 退出
    if (auto_collect && auto_target > 0 && static_cast<int>(image_points.size()) >= auto_target) {
      tools::logger()->info("已采集 {} 个样本，自动开始标定...", image_points.size());
      if (do_calibrate()) {
        do_write();
        tools::logger()->info("全自动标定完成，退出。");
        break;
      }
    }

    cv::Mat display = show_undistort && calibrated ? cv::Mat() : frame.clone();
    if (show_undistort && calibrated) {
      cv::Mat undistorted;
      cv::undistort(frame, undistorted, camera_matrix, dist_coeffs);
      cv::hconcat(frame, undistorted, display);
      tools::draw_text(display, "left: raw  right: undistorted", {20, 40}, {0, 255, 255}, 0.8, 2);
    }

    cv::Mat board_view = display;
    if (show_undistort && calibrated) {
      board_view = display(cv::Rect(0, 0, frame.cols, frame.rows));
    }
    cv::drawChessboardCorners(board_view, used_size, corners, found);

    int y = 35;
    const int dy = 28;
    tools::draw_text(
      display, fmt::format("pattern: {} {}x{}  spacing: {:.1f}mm",
      pattern_type == PatternType::circles ? "circles" : "chessboard",
      pattern_cols, pattern_rows, spacing_mm), {20, y}, {255, 255, 255}, 0.7, 2);
    y += dy;
    tools::draw_text(
      display, fmt::format("found: {}  ({:d}x{:d})  samples: {}/{}  min: {}", found ? "YES" : "NO",
      used_size.width, used_size.height, image_points.size(), auto_target, min_samples), {20, y},
      found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
      0.7, 2);
    y += dy;
    tools::draw_text(
      display, fmt::format("auto_collect: {}  undistort: {}", auto_collect ? "ON" : "OFF",
      show_undistort ? "ON" : "OFF"), {20, y}, {255, 255, 255}, 0.7, 2);
    y += dy;

    if (calibrated) {
      tools::draw_text(
        display, fmt::format("reproj err: {:.4f}px", reprojection_error), {20, y},
        {0, 255, 255}, 0.7, 2);
      y += dy;
      tools::draw_text(
        display,
        fmt::format("fx {:.2f} fy {:.2f} cx {:.2f} cy {:.2f}",
        camera_matrix.at<double>(0, 0), camera_matrix.at<double>(1, 1),
        camera_matrix.at<double>(0, 2), camera_matrix.at<double>(1, 2)),
        {20, y}, {0, 255, 255}, 0.65, 2);
      y += dy;
    }

    tools::draw_text(
      display, "keys: s save  c calibrate  w write  a auto  u undistort  r reset  q quit",
      {20, y}, {200, 200, 200}, 0.6, 2);

    // 采集成功提示：绿框 + 居中大字，持续 0.7 秒
    const double since_capture =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_capture_time).count();
    if (since_capture < 0.7) {
      cv::rectangle(
        display, cv::Rect(0, 0, display.cols - 1, display.rows - 1), cv::Scalar(0, 255, 0), 10);
      const std::string msg = fmt::format("CAPTURED {}/{}", image_points.size(), auto_target);
      tools::draw_text(
        display, msg, {display.cols / 2 - 220, display.rows / 2}, {0, 255, 0}, 1.6, 4);
    }

    // 窗口默认按图像大小创建，可能比 X 屏幕还大而看不见；固定成 960x720 并移到左上角。
    cv::imshow("mono_camera_calibration", display);
    int key = cv::waitKey(1);
    if (key == 'q') break;

    if (key == 's') {
      if (!found) {
        tools::logger()->warn("Pattern not found, sample ignored");
        continue;
      }
      image_points.push_back(corners);
      object_points.push_back(build_object_points(used_size, spacing_mm));
      last_auto_sample = corners;
      last_auto_collect_time = std::chrono::steady_clock::now();
      last_capture_time = std::chrono::steady_clock::now();
      tools::logger()->info(
        "✅ 采集成功 {}/{}  ({}x{}) —— 移动板子继续采集", image_points.size(), auto_target,
        used_size.width, used_size.height);
      continue;
    }

    if (key == 'a') {
      auto_collect = !auto_collect;
      tools::logger()->info("Auto collect {}", auto_collect ? "enabled" : "disabled");
      continue;
    }

    if (key == 'u') {
      show_undistort = !show_undistort;
      continue;
    }

    if (key == 'r') {
      image_points.clear();
      object_points.clear();
      last_auto_sample.clear();
      camera_matrix.release();
      dist_coeffs.release();
      reprojection_error = 0.0;
      calibrated = false;
      show_undistort = false;
      tools::logger()->info("Calibration state reset");
      continue;
    }

    if (key == 'c') {
      do_calibrate();
      continue;
    }

    if (key == 'w') {
      do_write();
      continue;
    }
  }

  return 0;
}
