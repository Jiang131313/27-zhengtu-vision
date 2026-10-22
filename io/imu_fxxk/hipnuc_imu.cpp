#include "io/imu_fxxk/hipnuc_imu.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cstring>

namespace io
{

HipnucIMU::HipnucIMU(const std::string & port, int baudrate)
: port_(port), baudrate_(baudrate), fd_(-1)
{
  std::memset(&raw_, 0, sizeof(raw_));
}

HipnucIMU::~HipnucIMU()
{
  stop();
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool HipnucIMU::init()
{
  fd_ = open_ttyport(port_, baudrate_);
  if (fd_ < 0) {
    last_error_ = "failed to open serial port: " + port_;
    return false;
  }
  return true;
}

void HipnucIMU::start()
{
  if (running_) return;
  running_ = true;
  recv_thread_ = std::thread(&HipnucIMU::receive_loop, this);
}

void HipnucIMU::stop()
{
  if (!running_) return;
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
}

void HipnucIMU::set_data_callback(std::function<void(const HipnucData &)> callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  data_callback_ = std::move(callback);
}

bool HipnucIMU::latest(HipnucData & data) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (latest_data_.timestamp_us == 0) return false;
  data = latest_data_;
  return true;
}

bool HipnucIMU::is_online(int timeout_ms) const
{
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_).count();
  return elapsed < timeout_ms;
}

std::string HipnucIMU::last_error() const
{
  return last_error_;
}

int HipnucIMU::open_ttyport(const std::string & tty_port, int baud) const
{
  const int serial_port = open(tty_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_port < 0) {
    return -1;
  }

  struct termios2 tty;
  if (ioctl(serial_port, TCGETS2, &tty) != 0) {
    close(serial_port);
    return -1;
  }

  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = baud;
  tty.c_ospeed = baud;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;

  if (ioctl(serial_port, TCSETS2, &tty) != 0) {
    close(serial_port);
    return -1;
  }

  return serial_port;
}

void HipnucIMU::receive_loop()
{
  uint8_t buf[kBufSize];
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;

  while (running_) {
    const int rpoll = poll(&pfd, 1, 10);
    if (rpoll <= 0) continue;

    const int n = read(fd_, buf, sizeof(buf));
    if (n > 0) {
      parse_data(buf, static_cast<size_t>(n));
    }
  }
}

void HipnucIMU::parse_data(const uint8_t * buf, size_t n)
{
  for (size_t i = 0; i < n; ++i) {
    if (hipnuc_input(&raw_, buf[i]) <= 0) continue;

    HipnucData data;
    if (raw_.hi83.tag == 0x83) {
      data.protocol_tag = 0x83;
      const uint32_t bm = raw_.hi83.data_bitmap;
      if (bm & HI83_BMAP_QUAT) {
        data.quat[0] = raw_.hi83.quat[0];
        data.quat[1] = raw_.hi83.quat[1];
        data.quat[2] = raw_.hi83.quat[2];
        data.quat[3] = raw_.hi83.quat[3];
      }
      if (bm & HI83_BMAP_GYR_B) {
        data.gyro[0] = raw_.hi83.gyr_b[0];
        data.gyro[1] = raw_.hi83.gyr_b[1];
        data.gyro[2] = raw_.hi83.gyr_b[2];
      }
      if (bm & HI83_BMAP_ACC_B) {
        data.acc[0] = raw_.hi83.acc_b[0];
        data.acc[1] = raw_.hi83.acc_b[1];
        data.acc[2] = raw_.hi83.acc_b[2];
      }
      if (bm & HI83_BMAP_RPY) {
        data.rpy[0] = raw_.hi83.rpy[0] * kDegToRad;
        data.rpy[1] = raw_.hi83.rpy[1] * kDegToRad;
        data.rpy[2] = raw_.hi83.rpy[2] * kDegToRad;
      }
      if (bm & HI83_BMAP_TEMPERATURE) {
        data.temperature = raw_.hi83.temperature;
      }
    } else if (raw_.hi91.tag == 0x91) {
      data.protocol_tag = 0x91;
      data.quat[0] = raw_.hi91.quat[0];
      data.quat[1] = raw_.hi91.quat[1];
      data.quat[2] = raw_.hi91.quat[2];
      data.quat[3] = raw_.hi91.quat[3];
      data.gyro[0] = raw_.hi91.gyr[0] * kDegToRad;
      data.gyro[1] = raw_.hi91.gyr[1] * kDegToRad;
      data.gyro[2] = raw_.hi91.gyr[2] * kDegToRad;
      data.acc[0] = raw_.hi91.acc[0] * kGravity;
      data.acc[1] = raw_.hi91.acc[1] * kGravity;
      data.acc[2] = raw_.hi91.acc[2] * kGravity;
      data.rpy[0] = raw_.hi91.roll * kDegToRad;
      data.rpy[1] = raw_.hi91.pitch * kDegToRad;
      data.rpy[2] = raw_.hi91.yaw * kDegToRad;
      data.temperature = static_cast<float>(raw_.hi91.temp);
    } else {
      continue;
    }

    data.timestamp_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_data_ = data;
      last_update_time_ = std::chrono::steady_clock::now();
    }

    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (data_callback_) data_callback_(data);
  }
}

}  // namespace io
