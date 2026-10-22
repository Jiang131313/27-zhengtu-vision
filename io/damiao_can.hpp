                                                   #ifndef IO__DAMIAO_CAN_HPP
#define IO__DAMIAO_CAN_HPP

#include "io/can_backend.hpp"

#include <linux/can.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include "io/damiao/pub_user.h"

#include <cstring>
#include <functional>
#include <memory>
#include <string>

namespace io
{
// 使用达妙官方 SDK 直接访问 DM-USB2FDCAN，无需 slcand/slcan0
class DamiaoCAN : public ICan
{
public:
  DamiaoCAN(
    const std::string &, std::function<void(const can_frame &)> rx_handler,
    int can_bitrate = 1000000, int can_channel = 0);
  ~DamiaoCAN() override;

  void write(can_frame * frame) const override;

private:
  static void rec_callback_static(usb_rx_frame_t * rec_frame);
  static void sent_callback_static(usb_rx_frame_t * sent_frame);
  static void err_callback_static(usb_rx_frame_t * err_frame);
  void on_rec(usb_rx_frame_t * rec_frame);
  void on_sent(usb_rx_frame_t * sent_frame);
  void on_err(usb_rx_frame_t * err_frame);

  std::function<void(const can_frame &)> rx_handler_;
  void * handle_ = nullptr;   // damiao_handle*
  void * dev_ = nullptr;      // device_handle*
  int channel_ = 0;
  bool debug_ = false;
  bool opened_ = false;
  mutable std::atomic<std::uint64_t> tx_request_count_{0};
  std::atomic<std::uint64_t> tx_sent_count_{0};
  std::atomic<std::uint64_t> rx_count_{0};
  std::atomic<std::uint64_t> err_count_{0};
  mutable std::chrono::steady_clock::time_point last_tx_debug_log_{};
};

}  // namespace io

#endif
