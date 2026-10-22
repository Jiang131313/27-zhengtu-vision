#include "io/imu_fxxk/hipnuc_pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace io
{

HipnucPoseBuffer::HipnucPoseBuffer(
  const std::string & port, int baudrate, std::chrono::milliseconds startup_timeout)
: imu_(port, baudrate)
{
  if (!imu_.init()) {
    throw std::runtime_error("HipnucIMU init failed: " + imu_.last_error());
  }

  imu_.set_data_callback([this](const HipnucData & data) { on_data(data); });
  imu_.start();

  const auto deadline = Clock::now() + startup_timeout;
  while (Clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!buffer_.empty()) return;
    }
    std::this_thread::sleep_for(20ms);
  }

  throw std::runtime_error("HipnucIMU started but no valid quaternion packet was received");
}

HipnucPoseBuffer::~HipnucPoseBuffer()
{
  imu_.stop();
}

Eigen::Quaterniond HipnucPoseBuffer::imu_at(Clock::time_point timestamp) const
{
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  if (!imu_at(timestamp, q)) {
    throw std::runtime_error("HipnucIMU has no buffered quaternion");
  }
  return q;
}

bool HipnucPoseBuffer::imu_at(Clock::time_point timestamp, Eigen::Quaterniond & q) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) return false;

  if (timestamp <= buffer_.front().timestamp) {
    q = buffer_.front().q;
    return true;
  }
  if (timestamp >= buffer_.back().timestamp) {
    q = buffer_.back().q;
    return true;
  }

  for (size_t i = 1; i < buffer_.size(); ++i) {
    if (buffer_[i].timestamp < timestamp) continue;

    const auto & a = buffer_[i - 1];
    const auto & b = buffer_[i];
    const std::chrono::duration<double> t_ab = b.timestamp - a.timestamp;
    if (t_ab.count() <= 0.0) {
      q = b.q;
      return true;
    }

    const std::chrono::duration<double> t_ac = timestamp - a.timestamp;
    const double ratio = std::clamp(t_ac / t_ab, 0.0, 1.0);
    q = a.q.slerp(ratio, b.q).normalized();
    return true;
  }

  q = buffer_.back().q;
  return true;
}

bool HipnucPoseBuffer::is_online(int timeout_ms) const
{
  return imu_.is_online(timeout_ms);
}

bool HipnucPoseBuffer::valid_quaternion(const Eigen::Quaterniond & q)
{
  return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
         std::isfinite(q.z()) && q.norm() > 1e-6;
}

HipnucPoseBuffer::Clock::time_point HipnucPoseBuffer::time_from_us(uint64_t timestamp_us)
{
  return Clock::time_point(
    std::chrono::duration_cast<Clock::duration>(std::chrono::microseconds(timestamp_us)));
}

void HipnucPoseBuffer::on_data(const HipnucData & data)
{
  Eigen::Quaterniond q(data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
  if (!valid_quaternion(q)) return;

  TimedQuat item;
  item.q = q.normalized();
  item.timestamp = time_from_us(data.timestamp_us);

  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.push_back(item);
  while (buffer_.size() > kMaxBufferSize) {
    buffer_.pop_front();
  }
}

}  // namespace io
