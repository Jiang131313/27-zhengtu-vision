#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>

#include "io/hikrobot/hikrobot.hpp"
#include "io/imu_huaner/imu_huaner.hpp"
#include "tools/yaml.hpp"

namespace
{
std::atomic_bool g_stop{false};

void signal_handler(int) { g_stop = true; }

std::string now_label()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

int64_t steady_ns(std::chrono::steady_clock::time_point timestamp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();
}

struct Args
{
  std::string config = "configs/demo.yaml";
  std::string target_config = "calibration/kalibr_config.yaml";
  std::string output_dir;
  double duration_s = 120.0;
  double image_hz = 10.0;
  bool preview = false;
};

Args parse_args(int argc, char ** argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + name);
      return argv[++i];
    };

    if (arg == "--config" || arg == "-c") {
      args.config = require_value(arg);
    } else if (arg == "--target-config") {
      args.target_config = require_value(arg);
    } else if (arg == "--output" || arg == "-o") {
      args.output_dir = require_value(arg);
    } else if (arg == "--duration") {
      args.duration_s = std::stod(require_value(arg));
    } else if (arg == "--image-hz") {
      args.image_hz = std::stod(require_value(arg));
    } else if (arg == "--preview") {
      args.preview = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: calibrate [--config configs/demo.yaml] "
                   "[--target-config calibration/kalibr_config.yaml] "
                   "[--output DIR] [--duration SEC] [--image-hz HZ] [--preview]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return args;
}

io::HuanerIMUOptions kalibr_imu_options(int configured_baudrate)
{
  io::HuanerIMUOptions options;
  options.configure_output = true;
  options.auto_detect_baudrate = true;
  options.target_baudrate = configured_baudrate >= 115200 ? configured_baudrate : 115200;
  options.output_rate_hz = 200;
  options.output_content =
    io::kHuanerOutputAcc | io::kHuanerOutputGyro | io::kHuanerOutputQuaternion |
    io::kHuanerOutputAngle;
  options.save_configuration = true;
  options.probe_timeout = std::chrono::milliseconds(300);
  return options;
}

struct ImageSaveJob
{
  cv::Mat image;
  std::filesystem::path path;
};
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 2;
  }

  const auto yaml = tools::load(args.config);
  const double exposure_ms = tools::read<double>(yaml, "exposure_ms");
  const double gain = tools::read<double>(yaml, "gain");
  const std::string vid_pid = tools::read<std::string>(yaml, "vid_pid");
  const std::string imu_port = tools::read<std::string>(yaml, "imu_port");
  const int imu_baudrate = tools::read<int>(yaml, "imu_baudrate");

  const auto target_yaml = tools::load(args.target_config);
  const int target_cols = target_yaml["target"]["cols"].as<int>();
  const int target_rows = target_yaml["target"]["rows"].as<int>();
  const cv::Size pattern_size(target_cols - 1, target_rows - 1);

  if (args.output_dir.empty()) {
    args.output_dir = "calibration/output/dataset_" + now_label();
  }
  if (args.image_hz <= 0.0) {
    std::cerr << "--image-hz must be > 0\n";
    return 2;
  }

  const std::filesystem::path dataset_dir(args.output_dir);
  const std::filesystem::path cam_dir = dataset_dir / "cam0";
  std::filesystem::create_directories(cam_dir);

  std::ofstream imu_csv(dataset_dir / "imu0.csv");
  if (!imu_csv) {
    std::cerr << "failed to open " << (dataset_dir / "imu0.csv") << "\n";
    return 1;
  }
  imu_csv << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n";

  std::atomic<int64_t> imu_count{0};
  std::atomic<int64_t> image_written_count{0};
  std::atomic_bool image_write_failed{false};

  std::unique_ptr<io::HuanerIMU> imu;
  try {
    imu = std::make_unique<io::HuanerIMU>(
      imu_port, static_cast<uint32_t>(imu_baudrate), kalibr_imu_options(imu_baudrate));
  } catch (const std::exception & e) {
    std::cerr << "IMU init failed: " << e.what() << "\n";
    return 1;
  }

  io::HikRobot camera(exposure_ms, gain, vid_pid);

  std::mutex image_queue_mutex;
  std::condition_variable image_queue_cv;
  std::deque<ImageSaveJob> image_queue;
  std::atomic_bool image_saving{true};

  std::thread image_writer([&]() {
    const std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 0};
    while (true) {
      ImageSaveJob job;
      {
        std::unique_lock<std::mutex> lock(image_queue_mutex);
        image_queue_cv.wait(
          lock, [&]() { return !image_queue.empty() || !image_saving || g_stop; });
        if (image_queue.empty()) {
          if (!image_saving || g_stop) break;
          continue;
        }
        job = std::move(image_queue.front());
        image_queue.pop_front();
      }
      if (!cv::imwrite(job.path.string(), job.image, png_params)) {
        std::cerr << "failed to write " << job.path << "\n";
        image_write_failed = true;
        g_stop = true;
        break;
      }
      ++image_written_count;
    }
  });

  std::atomic_bool imu_recording{true};
  std::thread imu_thread([&]() {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    constexpr double kGravity = 9.80665;
    std::chrono::steady_clock::time_point last_motion_timestamp{};

    while (imu_recording && !g_stop) {
      io::HuanerIMUData data;
      if (imu->latest(data) && data.motion_timestamp != std::chrono::steady_clock::time_point{} &&
          data.motion_timestamp != last_motion_timestamp) {
        last_motion_timestamp = data.motion_timestamp;
        const int64_t timestamp_ns = steady_ns(data.motion_timestamp);
        const Eigen::Vector3d gyro_rad_s = data.gyro_dps * kDegToRad;
        const Eigen::Vector3d acc_m_s2 = data.acc_g * kGravity;

        imu_csv << timestamp_ns << ',' << gyro_rad_s.x() << ',' << gyro_rad_s.y() << ','
                << gyro_rad_s.z() << ',' << acc_m_s2.x() << ',' << acc_m_s2.y() << ','
                << acc_m_s2.z() << '\n';
        ++imu_count;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  const auto start = std::chrono::steady_clock::now();
  const auto frame_period = std::chrono::duration<double>(1.0 / args.image_hz);
  auto next_frame_time = start;
  auto next_status_time = start;
  int64_t image_count = 0;
  int exit_code = 0;

  std::cout << fmt::format("Recording Kalibr dataset to {}\n", args.output_dir);
  std::cout << fmt::format("Camera vid_pid={} exposure_ms={} gain={}\n", vid_pid, exposure_ms, gain);
  std::cout << fmt::format("IMU port={} baudrate={}\n", imu_port, imu_baudrate);
  std::cout << fmt::format("Checkerboard target {}x{} (inner {}x{})\n", target_cols, target_rows,
    pattern_size.width, pattern_size.height);
  if (args.preview) {
    std::cout << "Preview window enabled. Press 'q' or ESC in the window to stop.\n";
  } else {
    std::cout << "Press Ctrl-C to stop.\n";
  }

  while (!g_stop) {
    if (args.duration_s > 0.0) {
      const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= args.duration_s) break;
    }

    cv::Mat image;
    std::chrono::steady_clock::time_point camera_timestamp;
    camera.read(image, camera_timestamp);
    if (image.empty()) continue;

    const auto now = std::chrono::steady_clock::now();

    if (now >= next_status_time) {
      std::cout << fmt::format(
                     "\rimages queued {} written {} imu {}", image_count,
                     image_written_count.load(), imu_count.load())
                << std::flush;
      next_status_time = now + std::chrono::seconds(1);
    }

    if (args.preview) {
      cv::Mat display;
      if (image.channels() == 1) {
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
      } else {
        display = image.clone();
      }

      std::vector<cv::Point2f> corners;
      cv::Mat gray;
      if (display.channels() == 3) {
        cv::cvtColor(display, gray, cv::COLOR_BGR2GRAY);
      } else {
        gray = display;
      }
      const bool found = cv::findChessboardCornersSB(
        gray, pattern_size, corners,
        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
      if (found) {
        cv::drawChessboardCorners(display, pattern_size, corners, found);
      }

      io::HuanerIMUData imu_data;
      if (imu->latest(imu_data)) {
        const auto & rpy = imu_data.rpy_deg;
        const std::string text = fmt::format("RPY(XYZ): {:.2f} {:.2f} {:.2f}", rpy.x(), rpy.y(), rpy.z());
        cv::putText(
          display, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
          cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      }

      cv::imshow("Kalibr Preview", display);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        g_stop = true;
      }
    }

    if (now < next_frame_time) continue;
    next_frame_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);

    const int64_t timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(camera_timestamp.time_since_epoch()).count();
    const auto image_path = cam_dir / fmt::format("{}.png", timestamp_ns);
    {
      std::lock_guard<std::mutex> lock(image_queue_mutex);
      image_queue.push_back({image, image_path});
    }
    image_queue_cv.notify_one();
    ++image_count;

    if (image_write_failed) {
      exit_code = 1;
      break;
    }
  }

  imu_recording = false;
  if (imu_thread.joinable()) imu_thread.join();

  image_saving = false;
  image_queue_cv.notify_one();
  if (image_writer.joinable()) image_writer.join();

  imu_csv.flush();
  if (image_write_failed) exit_code = 1;

  std::cout << "\n";
  std::cout << fmt::format(
    "Done. images={} imu_rows={} dataset={}\n", image_written_count.load(), imu_count.load(),
    args.output_dir);
  return exit_code;
}
