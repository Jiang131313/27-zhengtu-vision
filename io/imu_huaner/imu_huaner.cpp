#include "io/imu_huaner/imu_huaner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"

namespace io {

namespace {
constexpr uint8_t kRegSave = 0x00;
constexpr uint8_t kRegRsw = 0x02;
constexpr uint8_t kRegRate = 0x03;
constexpr uint8_t kRegBaud = 0x04;
constexpr uint8_t kRegKey = 0x69;
constexpr uint16_t kKeyUnlock = 0xB588;
constexpr uint16_t kSaveParam = 0x0000;

void add_unique_baudrate(std::vector<uint32_t>& baudrates, uint32_t baudrate) {
  if (baudrate == 0) return;
  if (std::find(baudrates.begin(), baudrates.end(), baudrate) == baudrates.end()) {
    baudrates.push_back(baudrate);
  }
}
}  // namespace

HuanerIMU::HuanerIMU(const std::string& port, uint32_t baudrate, HuanerIMUOptions options)
    : port_(port), baudrate_(baudrate), queue_(5000) {
  init_serial(options);
  if (options.configure_output) {
    configure_output(options);
  }
  receive_thread_ = std::thread(&HuanerIMU::receive_loop, this);
  tools::logger()->info("[HuanerIMU] initialized");
}

HuanerIMU::~HuanerIMU() {
  stop_thread_ = true;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void HuanerIMU::init_serial(const HuanerIMUOptions& options) {
  if (!options.auto_detect_baudrate) {
    open_serial(baudrate_);
    return;
  }

  std::vector<uint32_t> candidates;
  add_unique_baudrate(candidates, baudrate_);
  add_unique_baudrate(candidates, options.target_baudrate);
  add_unique_baudrate(candidates, 115200);
  add_unique_baudrate(candidates, 230400);
  add_unique_baudrate(candidates, 460800);
  add_unique_baudrate(candidates, 921600);
  add_unique_baudrate(candidates, 9600);

  for (const auto candidate : candidates) {
    try {
      open_serial(candidate);
      frame_pos_ = 0;
      if (probe_frames(options.probe_timeout)) {
        baudrate_ = candidate;
        tools::logger()->info("[HuanerIMU] detected active baudrate {}", baudrate_);
        return;
      }
      if (serial_.isOpen()) {
        serial_.close();
      }
    } catch (const std::exception& e) {
      tools::logger()->warn(
          "[HuanerIMU] baudrate probe failed: port={} baudrate={} error={}", port_, candidate,
          e.what());
      if (serial_.isOpen()) {
        serial_.close();
      }
    }
  }

  tools::logger()->warn(
      "[HuanerIMU] baudrate auto-detect failed; opening configured baudrate {}", baudrate_);
  open_serial(baudrate_);
}

void HuanerIMU::open_serial(uint32_t baudrate) {
  try {
    if (serial_.isOpen()) {
      serial_.close();
    }
    serial_.setPort(port_);
    serial_.setBaudrate(baudrate);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(5);
    serial_.setTimeout(timeout);
    serial_.open();
    serial_.flushInput();
    baudrate_ = baudrate;
    tools::logger()->info("[HuanerIMU] serial port {} opened at {}", port_, baudrate_);
  } catch (const serial::IOException& e) {
    throw std::runtime_error("[HuanerIMU] failed to open serial port " + port_);
  }
}

void HuanerIMU::reopen_serial(uint32_t baudrate) {
  if (serial_.isOpen()) {
    serial_.close();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  open_serial(baudrate);
}

bool HuanerIMU::probe_frames(std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto previous_update = last_update_time_;
  std::array<uint8_t, 256> buf{};

  while (std::chrono::steady_clock::now() < deadline) {
    const size_t n = serial_.read(buf.data(), buf.size());
    for (size_t i = 0; i < n; ++i) {
      input_byte(buf[i]);
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (last_update_time_ != previous_update &&
          last_update_time_ != std::chrono::steady_clock::time_point{}) {
        return true;
      }
    }

    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  return false;
}

void HuanerIMU::configure_output(const HuanerIMUOptions& options) {
  if (!serial_.isOpen()) {
    throw std::runtime_error("[HuanerIMU] serial port is not open");
  }

  const int rate_value = output_rate_value(options.output_rate_hz);
  if (rate_value < 0) {
    throw std::runtime_error("[HuanerIMU] unsupported output rate: " +
                             std::to_string(options.output_rate_hz));
  }

  if (options.target_baudrate != baudrate_) {
    const int baud_index = baudrate_index(options.target_baudrate);
    if (baud_index < 0) {
      throw std::runtime_error("[HuanerIMU] unsupported target baudrate: " +
                               std::to_string(options.target_baudrate));
    }

    write_reg(kRegKey, kKeyUnlock);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    write_reg(kRegBaud, static_cast<uint16_t>(baud_index));
    tools::logger()->info(
        "[HuanerIMU] switching baudrate: {} -> {}", baudrate_, options.target_baudrate);
    reopen_serial(options.target_baudrate);
  }

  write_reg(kRegKey, kKeyUnlock);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  write_reg(kRegRsw, options.output_content);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  write_reg(kRegRate, static_cast<uint16_t>(rate_value));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (options.save_configuration) {
    write_reg(kRegSave, kSaveParam);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  serial_.flushInput();
  frame_pos_ = 0;

  tools::logger()->info(
      "[HuanerIMU] configured output: baudrate={} rate={}Hz content_mask=0x{:03X}",
      baudrate_, options.output_rate_hz, options.output_content);
}

void HuanerIMU::write_reg(uint8_t reg, uint16_t value) {
  const uint8_t packet[5] = {
      0xFF, 0xAA, reg, static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF)};
  serial_.write(packet, sizeof(packet));
}

void HuanerIMU::receive_loop() {
  std::array<uint8_t, 128> buf{};

  while (!stop_thread_) {
    try {
      if (!serial_.isOpen()) {
        tools::logger()->warn("[HuanerIMU] serial port is not open");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      const size_t n = serial_.read(buf.data(), buf.size());
      for (size_t i = 0; i < n; ++i) {
        input_byte(buf[i]);
      }
    } catch (const std::exception& e) {
      tools::logger()->warn("[HuanerIMU] read failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void HuanerIMU::input_byte(uint8_t byte) {
  if (frame_pos_ == 0) {
    if (byte != 0x55) return;
    frame_buf_[frame_pos_++] = byte;
    return;
  }

  if (frame_pos_ == 1 && byte == 0x55) {
    frame_buf_[0] = byte;
    frame_pos_ = 1;
    return;
  }

  frame_buf_[frame_pos_++] = byte;
  if (frame_pos_ < kFrameSize) return;

  if (checksum_ok(frame_buf_)) {
    parse_frame(frame_buf_);
  }

  frame_pos_ = 0;
}

void HuanerIMU::parse_frame(const std::array<uint8_t, kFrameSize>& frame) {
  HuanerIMUData data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = latest_data_;
  }

  const uint8_t type = frame[1];
  if (type == 0x51) {
    data.acc_g = {read_i16(frame[2], frame[3]) / 32768.0 * 16.0,
                  read_i16(frame[4], frame[5]) / 32768.0 * 16.0,
                  read_i16(frame[6], frame[7]) / 32768.0 * 16.0};
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_ = data;
    last_update_time_ = now;
    return;
  }

  if (type == 0x52) {
    data.gyro_dps = {read_i16(frame[2], frame[3]) / 32768.0 * 2000.0,
                     read_i16(frame[4], frame[5]) / 32768.0 * 2000.0,
                     read_i16(frame[6], frame[7]) / 32768.0 * 2000.0};
    const auto now = std::chrono::steady_clock::now();
    data.motion_timestamp = now;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_ = data;
    last_update_time_ = now;
    return;
  }

  if (type == 0x53) {
    data.rpy_deg = {read_i16(frame[2], frame[3]) / 32768.0 * 180.0,
                    read_i16(frame[4], frame[5]) / 32768.0 * 180.0,
                    read_i16(frame[6], frame[7]) / 32768.0 * 180.0};
    data.q = euler_to_quaternion(data.rpy_deg);
  } else if (type == 0x59) {
    data.q = Eigen::Quaterniond(
        read_i16(frame[2], frame[3]) / 32768.0, read_i16(frame[4], frame[5]) / 32768.0,
        read_i16(frame[6], frame[7]) / 32768.0, read_i16(frame[8], frame[9]) / 32768.0);
    if (data.q.norm() < 1e-6) return;
    data.q.normalize();
  } else {
    return;
  }

  data.timestamp = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_ = data;
    last_update_time_ = data.timestamp;
  }
  queue_.push({data.q, data.timestamp});
}

Eigen::Quaterniond HuanerIMU::imu_at(std::chrono::steady_clock::time_point timestamp) {
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  return q_a.slerp(t_ac / t_ab, q_b).normalized();
}

bool HuanerIMU::latest(HuanerIMUData& data) const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (latest_data_.timestamp.time_since_epoch().count() == 0) return false;
  data = latest_data_;
  return true;
}

bool HuanerIMU::is_online(int timeout_ms) const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (last_update_time_.time_since_epoch().count() == 0) return false;
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_).count();
  return elapsed < timeout_ms;
}

Eigen::Quaterniond HuanerIMU::euler_to_quaternion(const Eigen::Vector3d& rpy_deg) const {
  constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;
  Eigen::Quaterniond q = Eigen::AngleAxisd(rpy_deg.z() * deg_to_rad, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy_deg.y() * deg_to_rad, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy_deg.x() * deg_to_rad, Eigen::Vector3d::UnitX());
  q.normalize();
  return q;
}

bool HuanerIMU::checksum_ok(const std::array<uint8_t, kFrameSize>& frame) {
  uint8_t sum = 0;
  for (size_t i = 0; i < kFrameSize - 1; ++i) {
    sum += frame[i];
  }
  return sum == frame[kFrameSize - 1];
}

int16_t HuanerIMU::read_i16(uint8_t low, uint8_t high) {
  return static_cast<int16_t>(static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
}

int HuanerIMU::baudrate_index(uint32_t baudrate) {
  switch (baudrate) {
    case 4800:
      return 1;
    case 9600:
      return 2;
    case 19200:
      return 3;
    case 38400:
      return 4;
    case 57600:
      return 5;
    case 115200:
      return 6;
    case 230400:
      return 7;
    case 460800:
      return 8;
    case 921600:
      return 9;
    default:
      return -1;
  }
}

int HuanerIMU::output_rate_value(int output_rate_hz) {
  switch (output_rate_hz) {
    case 1:
      return 0x03;
    case 2:
      return 0x04;
    case 5:
      return 0x05;
    case 10:
      return 0x06;
    case 20:
      return 0x07;
    case 50:
      return 0x08;
    case 100:
      return 0x09;
    case 125:
      return 0x0A;
    case 200:
      return 0x0B;
    default:
      return -1;
  }
}

}  // namespace io
