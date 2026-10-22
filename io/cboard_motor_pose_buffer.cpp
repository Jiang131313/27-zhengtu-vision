#include "cboard_motor_pose_buffer.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace io
{

CBoardMotorPoseBuffer::CBoardMotorPoseBuffer() = default;
CBoardMotorPoseBuffer::~CBoardMotorPoseBuffer() = default;

void CBoardMotorPoseBuffer::set_config(const Config & config)
{
  config_ = config;
}

void CBoardMotorPoseBuffer::on_motor_angle(const can_frame & frame,
                                            std::chrono::steady_clock::time_point timestamp)
{
  // 电控协议：yaw(int16 大端, data[0]=高 data[1]=低) + pitch(int16 大端, data[2]=高 data[3]=低)
  int16_t yaw_enc = (int16_t)(frame.data[0] << 8 | frame.data[1]);
  int16_t pitch_enc = (int16_t)(frame.data[2] << 8 | frame.data[3]);

  ++frame_count_;

  // pitch 是绝对角度（0=相对车水平，零点固定），只对 yaw 编码做零点标定。
  if (frame_count_ == kZeroCalibFrame) {
    if (yaw_enc != 0) {
      config_.motor_yaw_zero = yaw_enc;
    } else {
      --frame_count_;
      return;
    }
  }

  const double yaw_rad =
    config_.motor_yaw_direction *
    (yaw_enc - config_.motor_yaw_zero) *
    config_.motor_yaw_scale;

  const double pitch_rad =
    config_.motor_pitch_direction *
    (pitch_enc - config_.motor_pitch_zero) *
    config_.motor_pitch_scale;

  // ---- 调试打印（限流 1Hz）：把电控发来的原始值和解算结果都打出来 ----
  // 用途：确认 0x106 到底有没有发真实数据、零点标定是否生效、换算是否合理。
  {
    static auto last_dbg = std::chrono::steady_clock::now();
    static int dbg_frames = 0;
    ++dbg_frames;
    const auto now_dbg = std::chrono::steady_clock::now();
    const double dbg_dt = std::chrono::duration<double>(now_dbg - last_dbg).count();
    if (dbg_dt >= 1.0) {
      // 简单明了：解算出的 yaw/pitch（度） + 原始编码值 + 接收频率
      std::printf(
        "## rx yaw: %+.2fdeg   pitch: %+.2fdeg   | enc %d / %d | %.1fHz\n",
        yaw_rad * 180.0 / M_PI, pitch_rad * 180.0 / M_PI, static_cast<int>(yaw_enc),
        static_cast<int>(pitch_enc), dbg_frames / dbg_dt);
      std::fflush(stdout);
      dbg_frames = 0;
      last_dbg = now_dbg;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.push_back({yaw_rad, pitch_rad, timestamp});
}

Eigen::Quaterniond CBoardMotorPoseBuffer::imu_at(
  std::chrono::steady_clock::time_point timestamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty()) return Eigen::Quaterniond::Identity();

  auto it = buffer_.begin();
  while (it != buffer_.end() && it->timestamp <= timestamp) ++it;

  if (it == buffer_.begin()) {
    const auto & a = buffer_.front();
    Eigen::Quaterniond q = Eigen::AngleAxisd(a.yaw_rad, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(a.pitch_rad, Eigen::Vector3d::UnitY());
    q.normalize();
    return q;
  }

  if (it == buffer_.end()) {
    const auto & a = buffer_.back();
    Eigen::Quaterniond q = Eigen::AngleAxisd(a.yaw_rad, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(a.pitch_rad, Eigen::Vector3d::UnitY());
    q.normalize();
    return q;
  }

  const auto & a = *(it - 1);
  const auto & b = *it;

  double t_ab = std::chrono::duration<double>(b.timestamp - a.timestamp).count();
  double t_ac = std::chrono::duration<double>(timestamp - a.timestamp).count();
  if (t_ab <= 0) t_ab = 1e-9;
  double k = t_ac / t_ab;

  double yaw_diff = std::fmod(b.yaw_rad - a.yaw_rad + M_PI, 2 * M_PI) - M_PI;

  double yaw_interp = a.yaw_rad + yaw_diff * k;
  double pitch_interp = a.pitch_rad + (b.pitch_rad - a.pitch_rad) * k;

  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw_interp, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch_interp, Eigen::Vector3d::UnitY());
  q.normalize();
  return q;
}

}  // namespace io
