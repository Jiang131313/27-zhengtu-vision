#ifndef IO__HIPNUC_IMU__HIPNUC_POSE_BUFFER_HPP
#define IO__HIPNUC_IMU__HIPNUC_POSE_BUFFER_HPP

#include <Eigen/Geometry>

#include <chrono>
#include <deque>
#include <mutex>
#include <string>

#include "io/imu_fxxk/hipnuc_imu.hpp"

namespace io
{

class HipnucPoseBuffer
{
public:
  explicit HipnucPoseBuffer(
    const std::string & port = "/dev/ttyACM0", int baudrate = 921600,
    std::chrono::milliseconds startup_timeout = std::chrono::seconds(3));
  ~HipnucPoseBuffer();

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
  static Clock::time_point time_from_us(uint64_t timestamp_us);

  void on_data(const HipnucData & data);

  HipnucIMU imu_;
  mutable std::mutex mutex_;
  std::deque<TimedQuat> buffer_;
};

}  // namespace io

#endif  // IO__HIPNUC_IMU__HIPNUC_POSE_BUFFER_HPP
