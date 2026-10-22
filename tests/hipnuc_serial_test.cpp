#include <chrono>
#include <thread>

#include "io/imu_fxxk/hipnuc_imu.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
  const int baudrate = argc > 2 ? std::stoi(argv[2]) : 921600;

  tools::Exiter exiter;
  io::HipnucIMU imu(port, baudrate);
  if (!imu.init()) {
    tools::logger()->error("HipnucIMU init failed: {}", imu.last_error());
    return 1;
  }

  imu.start();
  tools::logger()->info("Hipnuc serial test started. port={} baudrate={}", port, baudrate);

  while (!exiter.exit()) {
    io::HipnucData data;
    if (imu.latest(data)) {
      tools::logger()->info(
        "tag=0x{:02X} quat[wxyz]=[{:.4f}, {:.4f}, {:.4f}, {:.4f}] rpy[deg]=[{:.2f}, {:.2f}, {:.2f}] gyro[rad/s]=[{:.3f}, {:.3f}, {:.3f}] acc[m/s^2]=[{:.3f}, {:.3f}, {:.3f}] online={}",
        data.protocol_tag,
        data.quat[0], data.quat[1], data.quat[2], data.quat[3],
        data.rpy[0] * 57.295779513, data.rpy[1] * 57.295779513, data.rpy[2] * 57.295779513,
        data.gyro[0], data.gyro[1], data.gyro[2],
        data.acc[0], data.acc[1], data.acc[2],
        imu.is_online());
    } else {
      tools::logger()->warn("No valid HiPNUC packet decoded yet");
    }
    std::this_thread::sleep_for(200ms);
  }

  imu.stop();
  return 0;
}
