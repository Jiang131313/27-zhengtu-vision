#ifndef IO__CBOARD_MOTOR_POSE_BUFFER_HPP
#define IO__CBOARD_MOTOR_POSE_BUFFER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <mutex>

#include "io/can_backend.hpp"

namespace io
{

class CBoardMotorPoseBuffer
{
public:
  struct Config
  {
    double motor_yaw_zero = 0;
    double motor_pitch_zero = 0;
    double motor_yaw_scale = 0.00076699;   // 2*pi/8192: ecd(编码) -> 弧度
    double motor_pitch_scale = 0.01745329; // pi/180: 度 -> 弧度
    int motor_yaw_direction = 1;
    int motor_pitch_direction = 1;
  };

  explicit CBoardMotorPoseBuffer();
  ~CBoardMotorPoseBuffer();

  void set_config(const Config & config);

  void on_motor_angle(const can_frame & frame,
                      std::chrono::steady_clock::time_point timestamp);

  Eigen::Quaterniond imu_at(
    std::chrono::steady_clock::time_point timestamp) const;

private:
  struct TimedMotorAngle
  {
    double yaw_rad;
    double pitch_rad;
    std::chrono::steady_clock::time_point timestamp;
  };

  Config config_;
  mutable std::deque<TimedMotorAngle> buffer_;
  mutable std::mutex mutex_;
  int frame_count_ = 0;
  static constexpr int kZeroCalibFrame = 50;
};

}  // namespace io

#endif  // IO__CBOARD_MOTOR_POSE_BUFFER_HPP
