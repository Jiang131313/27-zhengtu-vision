#ifndef IO__IMU_HUANER__IMU_HUANER_HPP
#define IO__IMU_HUANER__IMU_HUANER_HPP

#include <serial/serial.h>

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io {

constexpr uint16_t kHuanerOutputTime = 0x0001;
constexpr uint16_t kHuanerOutputAcc = 0x0002;
constexpr uint16_t kHuanerOutputGyro = 0x0004;
constexpr uint16_t kHuanerOutputAngle = 0x0008;
constexpr uint16_t kHuanerOutputQuaternion = 0x0200;

struct HuanerIMUOptions {
  bool configure_output = false;
  bool auto_detect_baudrate = false;
  uint32_t target_baudrate = 115200;
  int output_rate_hz = 200;
  uint16_t output_content = kHuanerOutputQuaternion;
  bool save_configuration = true;
  std::chrono::milliseconds probe_timeout{300};
};

struct HuanerIMUData {
  Eigen::Vector3d acc_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_dps = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_deg = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point motion_timestamp{};
  std::chrono::steady_clock::time_point timestamp{};
};

class HuanerIMU {
 public:
  explicit HuanerIMU(
      const std::string& port = "/dev/ttyUSB0", uint32_t baudrate = 9600,
      HuanerIMUOptions options = {});
  ~HuanerIMU();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  bool latest(HuanerIMUData& data) const;
  bool is_online(int timeout_ms = 100) const;

 private:
  struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  static constexpr size_t kFrameSize = 11;

  void init_serial(const HuanerIMUOptions& options);
  void open_serial(uint32_t baudrate);
  void reopen_serial(uint32_t baudrate);
  bool probe_frames(std::chrono::milliseconds timeout);
  void configure_output(const HuanerIMUOptions& options);
  void write_reg(uint8_t reg, uint16_t value);
  void receive_loop();
  void input_byte(uint8_t byte);
  void parse_frame(const std::array<uint8_t, kFrameSize>& frame);
  Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& rpy_deg) const;

  static bool checksum_ok(const std::array<uint8_t, kFrameSize>& frame);
  static int16_t read_i16(uint8_t low, uint8_t high);
  static int baudrate_index(uint32_t baudrate);
  static int output_rate_value(int output_rate_hz);

  std::string port_;
  uint32_t baudrate_;
  serial::Serial serial_;

  std::thread receive_thread_;
  std::atomic<bool> stop_thread_{false};

  std::array<uint8_t, kFrameSize> frame_buf_{};
  size_t frame_pos_ = 0;

  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_{};
  IMUData data_behind_{};

  mutable std::mutex data_mutex_;
  HuanerIMUData latest_data_{};
  std::chrono::steady_clock::time_point last_update_time_{};
};

}  // namespace io

#endif  // IO__IMU_HUANER__IMU_HUANER_HPP
