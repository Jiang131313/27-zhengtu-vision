#ifndef IO__HIPNUC_IMU__HIPNUC_IMU_HPP
#define IO__HIPNUC_IMU__HIPNUC_IMU_HPP

#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

extern "C" {
#include "hipnuc_dec.h"
}

namespace io
{

struct HipnucData
{
  float quat[4] = {1.0F, 0.0F, 0.0F, 0.0F};  // w x y z
  float gyro[3] = {0.0F, 0.0F, 0.0F};        // rad/s
  float acc[3] = {0.0F, 0.0F, 0.0F};         // m/s^2
  float rpy[3] = {0.0F, 0.0F, 0.0F};         // roll pitch yaw, rad
  uint64_t timestamp_us = 0;
  float temperature = 0.0F;
  uint8_t protocol_tag = 0;
};

class HipnucIMU
{
public:
  explicit HipnucIMU(const std::string & port = "/dev/ttyACM0", int baudrate = 921600);
  ~HipnucIMU();

  bool init();
  void start();
  void stop();

  void set_data_callback(std::function<void(const HipnucData &)> callback);
  bool latest(HipnucData & data) const;
  bool is_online(int timeout_ms = 100) const;
  std::string last_error() const;

private:
  static constexpr float kGravity = 9.8F;
  static constexpr float kDegToRad = 0.01745329F;
  static constexpr size_t kBufSize = 1024;

  int open_ttyport(const std::string & tty_port, int baud) const;
  void receive_loop();
  void parse_data(const uint8_t * buf, size_t n);

  std::string port_;
  int baudrate_;
  int fd_;
  hipnuc_raw_t raw_{};

  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex data_mutex_;
  HipnucData latest_data_{};
  std::chrono::steady_clock::time_point last_update_time_{};

  mutable std::mutex callback_mutex_;
  std::function<void(const HipnucData &)> data_callback_;

  std::string last_error_;
};

}  // namespace io

#endif
