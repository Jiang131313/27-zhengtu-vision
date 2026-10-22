#include "io/damiao_can.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

#include "tools/logger.hpp"

namespace
{
std::atomic<bool> g_stop{false};

void on_signal(int)
{
  g_stop = true;
}

std::string format_frame(const can_frame & frame)
{
  std::ostringstream oss;
  oss << "id=0x" << std::hex << std::uppercase << frame.can_id;
  oss << " dlc=" << std::dec << static_cast<int>(frame.can_dlc) << " data=";

  oss << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < frame.can_dlc && i < 8; ++i) {
    if (i > 0) oss << ' ';
    oss << std::setw(2) << static_cast<int>(frame.data[i]);
  }

  return oss.str();
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string selector = argc > 1 ? argv[1] : "damiao:1";

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::mutex print_mutex;
  std::atomic<uint64_t> frame_count{0};

  try {
    io::DamiaoCAN can(selector, [&](const can_frame & frame) {
      const auto count = ++frame_count;
      std::lock_guard<std::mutex> lock(print_mutex);
      tools::logger()->info("[{}] {}", count, format_frame(frame));
    });

    tools::logger()->info(
      "Damiao CAN dump started. selector={}, Ctrl+C to stop.",
      selector);

    auto last_count = uint64_t{0};
    while (!g_stop.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const auto current_count = frame_count.load();
      const auto delta = current_count - last_count;
      last_count = current_count;
      tools::logger()->info("rx {} frame(s) in last 1s, total={}", delta, current_count);
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Damiao CAN dump failed: {}", e.what());
    return 1;
  }

  tools::logger()->info("Damiao CAN dump stopped.");
  return 0;
}
