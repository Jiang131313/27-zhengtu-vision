#include <chrono>
#include <thread>

#include "io/imu_huaner/huaner_pose_buffer.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  const int baudrate = argc > 2 ? std::stoi(argv[2]) : 9600;

  try {
    tools::Exiter exiter;
    io::HuanerPoseBuffer imu(port, baudrate);
    tools::logger()->info("Huaner serial test started. port={} baudrate={}", port, baudrate);

    while (!exiter.exit()) {
      const auto timestamp = std::chrono::steady_clock::now();
      Eigen::Quaterniond q;
      if (imu.imu_at(timestamp, q)) {
        const Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.295779513;
        tools::logger()->info(
          "quat[wxyz]=[{:.4f}, {:.4f}, {:.4f}, {:.4f}] ypr[deg]=[{:.2f}, {:.2f}, {:.2f}] online={}",
          q.w(), q.x(), q.y(), q.z(), eulers[0], eulers[1], eulers[2], imu.is_online());
      } else {
        tools::logger()->warn("No buffered Huaner quaternion yet");
      }
      std::this_thread::sleep_for(200ms);
    }

    return 0;
  } catch (const std::exception & e) {
    tools::logger()->error("Huaner serial test failed: {}", e.what());
    return 1;
  }
}
