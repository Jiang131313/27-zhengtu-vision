#ifndef IO__IMU_HUANER__HUANER_POSE_BUFFER_HPP
#define IO__IMU_HUANER__HUANER_POSE_BUFFER_HPP

#include <Eigen/Geometry>

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "io/imu_huaner/imu_huaner.hpp"

namespace io
{

class HuanerPoseBuffer
{
public:
  explicit HuanerPoseBuffer(
    const std::string & port = "/dev/ttyUSB0", int baudrate = 9600,
    std::chrono::milliseconds startup_timeout = std::chrono::seconds(3),
    std::chrono::milliseconds poll_interval = std::chrono::milliseconds(1));
  ~HuanerPoseBuffer();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) const;
  bool imu_at(std::chrono::steady_clock::time_point timestamp, Eigen::Quaterniond & q) const;
  bool is_online(int timeout_ms = 200) const;

private:
  using Clock = std::chrono::steady_clock;

  struct TimedQuat
  {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Clock::time_point timestamp{};
  };

  static constexpr size_t kMaxBufferSize = 2000;

  static bool valid_quaternion(const Eigen::Quaterniond & q);

  void collect_loop(std::chrono::milliseconds poll_interval);

  HuanerIMU imu_;
  mutable std::mutex mutex_;
  std::deque<TimedQuat> buffer_;
  std::thread collect_thread_;
  bool stop_collect_ = false;
};

}  // namespace io

#endif  // IO__IMU_HUANER__HUANER_POSE_BUFFER_HPP
