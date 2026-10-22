#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

extern "C" {
#include "io/damiao/pub_user.h"
}

#include "tools/logger.hpp"

namespace
{
std::atomic<bool> g_stop{false};
std::atomic<uint64_t> g_frame_count{0};

void on_signal(int)
{
  g_stop = true;
}

std::string trim_copy(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return std::string();
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

bool is_all_digits(const std::string & text)
{
  if (text.empty()) return false;
  for (const char ch : text) {
    if (ch < '0' || ch > '9') return false;
  }
  return true;
}

int parse_positive_int(const std::string & text, const char * name)
{
  try {
    const int value = std::stoi(text);
    if (value <= 0) throw std::runtime_error("");
    return value;
  } catch (...) {
    throw std::runtime_error(std::string("invalid ") + name + ": " + text);
  }
}

int select_device_index(device_handle ** dev_list, int count, const std::string & selector)
{
  if (count <= 0) throw std::runtime_error("no device available");

  std::string sn = trim_copy(selector);
  int ordinal = 1;

  if (!sn.empty()) {
    const std::string prefix = "damiao:";
    if (sn.rfind(prefix, 0) == 0) {
      sn = trim_copy(sn.substr(prefix.size()));
    }

    if (is_all_digits(sn)) {
      ordinal = parse_positive_int(sn, "selector");
      sn.clear();
    }
  }

  if (!sn.empty()) {
    for (int i = 0; i < count; ++i) {
      char serial[256] = {0};
      device_get_serial_number(dev_list[i], serial, sizeof(serial));
      if (sn == serial) return i;
    }
    throw std::runtime_error("no device matched selector SN=" + sn);
  }

  const int index = ordinal - 1;
  if (index < 0 || index >= count) {
    throw std::runtime_error(
      "selector out of range, ordinal=" + std::to_string(ordinal) +
      " detected=" + std::to_string(count));
  }
  return index;
}

std::string format_frame(const usb_rx_frame_t * frame)
{
  std::ostringstream oss;
  oss << "id=0x" << std::hex << std::uppercase << frame->head.can_id
      << " ch=" << std::dec << static_cast<int>(frame->head.channel)
      << " dlc=" << static_cast<int>(frame->head.dlc)
      << " ext=" << static_cast<int>(frame->head.ext)
      << " fd=" << static_cast<int>(frame->head.canfd)
      << " brs=" << static_cast<int>(frame->head.brs)
      << " ack=" << static_cast<int>(frame->head.ack)
      << " data=";

  oss << std::hex << std::uppercase << std::setfill('0');
  const int len = frame->head.dlc <= 64 ? frame->head.dlc : 64;
  for (int i = 0; i < len; ++i) {
    if (i > 0) oss << ' ';
    oss << std::setw(2) << static_cast<int>(frame->payload[i]);
  }
  return oss.str();
}

void recv_hook(usb_rx_frame_t * frame)
{
  if (!frame) return;
  const auto count = ++g_frame_count;
  tools::logger()->info("[{}] {}", count, format_frame(frame));
}

void err_hook(usb_rx_frame_t * frame)
{
  if (!frame) return;
  tools::logger()->warn("err {}", format_frame(frame));
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string selector = argc > 1 ? argv[1] : "damiao:1";
  const int nominal_baud = argc > 2 ? parse_positive_int(argv[2], "nominal_baud") : 1000000;
  const int data_baud = argc > 3 ? parse_positive_int(argv[3], "data_baud") : nominal_baud;
  const bool is_fd = argc > 4 ? std::stoi(argv[4]) != 0 : false;

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  damiao_handle * handle = nullptr;
  device_handle * dev = nullptr;
  bool channel_opened = false;
  bool device_opened = false;

  try {
    handle = damiao_handle_create(DEV_USB2CANFD);
    if (!handle) throw std::runtime_error("damiao_handle_create failed");

    damiao_print_version(handle);

    const int found = damiao_handle_find_devices(handle);
    if (found <= 0) throw std::runtime_error("no CAN device found");

    device_handle * dev_list[16] = {nullptr};
    int count = 0;
    damiao_handle_get_devices(handle, dev_list, &count);
    if (count <= 0) throw std::runtime_error("damiao_handle_get_devices failed");

    tools::logger()->info("detected {} device(s)", count);
    for (int i = 0; i < count; ++i) {
      char serial[256] = {0};
      device_get_serial_number(dev_list[i], serial, sizeof(serial));
      tools::logger()->info("device[{}] sn={}", i, serial);
    }

    const int index = select_device_index(dev_list, count, selector);
    dev = dev_list[index];

    char selected_sn[256] = {0};
    device_get_serial_number(dev, selected_sn, sizeof(selected_sn));
    tools::logger()->info("selected device index={} sn={}", index, selected_sn);

    tools::logger()->info("calling device_open...");
    if (!device_open(dev)) throw std::runtime_error("device_open failed");
    device_opened = true;
    tools::logger()->info("device_open ok");

    tools::logger()->info(
      "calling device_channel_set_baud_with_sp(channel=0, nominal_baud={}, data_baud={}, is_fd={})...",
      nominal_baud,
      data_baud,
      is_fd);
    const bool set_ok =
      device_channel_set_baud_with_sp(dev, 0, is_fd, nominal_baud, data_baud, 0.75f, 0.75f);
    tools::logger()->info("device_channel_set_baud_with_sp returned {}", set_ok);
    if (!set_ok) {
      tools::logger()->warn(
        "device_channel_set_baud_with_sp returned false, continue to inspect actual channel baud");
    }

    tools::logger()->info("calling device_channel_get_baudrate...");
    device_baud_t baud = {};
    if (device_channel_get_baudrate(dev, 0, &baud)) {
      tools::logger()->info(
        "channel 0 baud after set: nominal={} data={} nominal_sp={} data_sp={}",
        baud.can_baudrate,
        baud.canfd_baudrate,
        baud.can_sp,
        baud.canfd_sp);
    } else {
      tools::logger()->warn("device_channel_get_baudrate failed");
    }

    tools::logger()->info("calling device_open_channel(0)...");
    if (!device_open_channel(dev, 0)) throw std::runtime_error("device_open_channel(0) failed");
    channel_opened = true;
    tools::logger()->info("device_open_channel(0) ok");

    device_hook_to_rec(dev, recv_hook);
    device_hook_to_err(dev, err_hook);

    tools::logger()->info(
      "Damiao baud dump started. selector={} sn={} nominal_baud={} data_baud={} is_fd={} Ctrl+C to stop.",
      selector,
      selected_sn,
      nominal_baud,
      data_baud,
      is_fd);

    uint64_t last_count = 0;
    while (!g_stop.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const auto current_count = g_frame_count.load();
      tools::logger()->info(
        "rx {} frame(s) in last 1s, total={}",
        current_count - last_count,
        current_count);
      last_count = current_count;
    }

    device_hook_to_rec(dev, nullptr);
    device_hook_to_err(dev, nullptr);
    if (channel_opened) device_close_channel(dev, 0);
    if (device_opened) device_close(dev);
    damiao_handle_destroy(handle);
  } catch (const std::exception & e) {
    if (dev) {
      device_hook_to_rec(dev, nullptr);
      device_hook_to_err(dev, nullptr);
    }
    if (channel_opened) device_close_channel(dev, 0);
    if (device_opened) device_close(dev);
    if (handle) damiao_handle_destroy(handle);
    tools::logger()->error("Damiao baud dump failed: {}", e.what());
    return 1;
  }

  tools::logger()->info("Damiao baud dump stopped.");
  return 0;
}
