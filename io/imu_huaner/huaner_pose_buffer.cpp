#include "io/imu_huaner/huaner_pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace io
{

namespace
{
HuanerIMUOptions pose_buffer_options(int configured_baudrate)
{
  HuanerIMUOptions options;
  options.configure_output = true;
  options.auto_detect_baudrate = true;
  options.target_baudrate = configured_baudrate >= 115200 ? configured_baudrate : 115200;
  options.output_rate_hz = 200;
  options.output_content = kHuanerOutputQuaternion;
  options.save_configuration = true;
  options.probe_timeout = std::chrono::milliseconds(300);
  return options;
}
}  // namespace

HuanerPoseBuffer::HuanerPoseBuffer(
  const std::string & port, int baudrate, std::chrono::milliseconds startup_timeout,
  std::chrono::milliseconds poll_interval)
: imu_(port, baudrate, pose_buffer_options(baudrate))
{
  collect_thread_ = std::thread(&HuanerPoseBuffer::collect_loop, this, poll_interval);

  const auto deadline = Clock::now() + startup_timeout;
  while (Clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!buffer_.empty()) return;
    }
    std::this_thread::sleep_for(20ms);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_collect_ = true;
  }
  if (collect_thread_.joinable()) {
    collect_thread_.join();
  }
  throw std::runtime_error("HuanerIMU started but no valid quaternion packet was received");
}

HuanerPoseBuffer::~HuanerPoseBuffer()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_collect_ = true;
  }
  if (collect_thread_.joinable()) {
    collect_thread_.join();
  }
}

Eigen::Quaterniond HuanerPoseBuffer::imu_at(Clock::time_point timestamp) const
{
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  if (!imu_at(timestamp, q)) {
    throw std::runtime_error("HuanerIMU has no buffered quaternion");
  }
  return q;
}

bool HuanerPoseBuffer::imu_at(Clock::time_point timestamp, Eigen::Quaterniond & q) const
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

bool HuanerPoseBuffer::is_online(int timeout_ms) const
{
  return imu_.is_online(timeout_ms);
}

bool HuanerPoseBuffer::valid_quaternion(const Eigen::Quaterniond & q)
{
  return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
         std::isfinite(q.z()) && q.norm() > 1e-6;
}

void HuanerPoseBuffer::collect_loop(std::chrono::milliseconds poll_interval)
{
  auto last_timestamp = Clock::time_point{};

  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_collect_) break;
    }

    HuanerIMUData data;
    if (imu_.latest(data) && data.timestamp != last_timestamp && valid_quaternion(data.q)) {
      TimedQuat item;
      item.q = data.q.normalized();
      item.timestamp = data.timestamp;

      std::lock_guard<std::mutex> lock(mutex_);
      buffer_.push_back(item);
      while (buffer_.size() > kMaxBufferSize) {
        buffer_.pop_front();
      }
      last_timestamp = data.timestamp;
      continue;
    }

    std::this_thread::sleep_for(poll_interval);
  }
}

}  // namespace io
