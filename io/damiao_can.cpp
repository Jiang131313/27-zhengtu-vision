#include "io/damiao_can.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <algorithm>
#include <cctype>

#include "tools/logger.hpp"

namespace io
{
// SDK 回调无 user_data，单例转发（CBoard 仅一个 CAN 实例）
static DamiaoCAN * s_rec_instance = nullptr;

namespace
{
bool is_all_digits(const std::string & text)
{
  return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char ch) {return std::isdigit(ch);});
}

std::string format_payload(const std::uint8_t * payload, const int len)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < len; ++i) {
    if (i > 0) oss << ' ';
    oss << std::setw(2) << static_cast<int>(payload[i]);
  }
  return oss.str();
}

std::string format_usb_frame(const usb_rx_frame_t * frame)
{
  if (!frame) return "<null>";
  const int len = frame->head.dlc <= 64 ? frame->head.dlc : 64;
  std::ostringstream oss;
  oss << "id=0x" << std::hex << std::uppercase << frame->head.can_id
      << std::dec << " ch=" << static_cast<int>(frame->head.channel)
      << " dlc=" << static_cast<int>(frame->head.dlc)
      << " ext=" << static_cast<int>(frame->head.ext)
      << " fd=" << static_cast<int>(frame->head.canfd)
      << " brs=" << static_cast<int>(frame->head.brs)
      << " ack=" << static_cast<int>(frame->head.ack)
      << " data=" << format_payload(frame->payload, len);
  return oss.str();
}

std::string trim_copy(const std::string & s)
{
  auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char ch) {return std::isspace(ch);});
  auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char ch) {return std::isspace(ch);}).base();
  if (begin >= end) return std::string();
  return std::string(begin, end);
}

bool env_enabled(const char * name)
{
  if (const char * value = std::getenv(name)) {
    const auto text = trim_copy(value);
    return !(text.empty() || text == "0" || text == "false" || text == "FALSE" || text == "off" ||
           text == "OFF");
  }
  return false;
}

int parse_human_device_ordinal(const std::string & raw, const char * source)
{
  const auto text = trim_copy(raw);
  if (text.empty()) {
    throw std::runtime_error(std::string("DamiaoCAN: empty ") + source);
  }

  int ordinal = 0;
  try {
    ordinal = std::stoi(text);
  } catch (...) {
    throw std::runtime_error(std::string("DamiaoCAN: invalid ") + source + ", expected positive integer");
  }

  if (ordinal <= 0) {
    throw std::runtime_error(std::string("DamiaoCAN: invalid ") + source + ", expected >= 1");
  }
  return ordinal;
}

std::string parse_selector_sn(const std::string & selector)
{
  const auto text = trim_copy(selector);
  if (text.empty()) {
    return std::string();
  }

  const std::string damiao_prefix = "damiao:";
  if (text.rfind(damiao_prefix, 0) == 0) {
    const auto body = trim_copy(text.substr(damiao_prefix.size()));
    if (body.empty() || is_all_digits(body)) {
      return std::string();
    }
    const std::string sn_colon_prefix = "sn:";
    const std::string sn_equal_prefix = "sn=";
    if (body.rfind(sn_colon_prefix, 0) == 0) {
      return trim_copy(body.substr(sn_colon_prefix.size()));
    }
    if (body.rfind(sn_equal_prefix, 0) == 0) {
      return trim_copy(body.substr(sn_equal_prefix.size()));
    }
    return body;
  }

  const std::string sn_colon_prefix = "sn:";
  const std::string sn_equal_prefix = "sn=";
  const std::string damiao_sn_prefix = "damiao_sn:";
  if (text.rfind(sn_colon_prefix, 0) == 0) {
    return trim_copy(text.substr(sn_colon_prefix.size()));
  }
  if (text.rfind(sn_equal_prefix, 0) == 0) {
    return trim_copy(text.substr(sn_equal_prefix.size()));
  }
  if (text.rfind(damiao_sn_prefix, 0) == 0) {
    return trim_copy(text.substr(damiao_sn_prefix.size()));
  }

  return std::string();
}

const char * device_type_name(const device_def_t type)
{
  switch (type) {
    case DEV_USB2CANFD:
      return "DEV_USB2CANFD";
    case DEV_USB2CANFD_DUAL:
      return "DEV_USB2CANFD_DUAL";
    case DEV_ECAT2CANFD:
      return "DEV_ECAT2CANFD";
    default:
      return "DEV_None";
  }
}
}  // namespace

DamiaoCAN::DamiaoCAN(
  const std::string & selector, std::function<void(const can_frame &)> rx_handler, const int can_bitrate,
  const int can_channel)
: rx_handler_(std::move(rx_handler)), channel_(can_channel), debug_(env_enabled("DM_CAN_DEBUG"))
{
  if (can_bitrate <= 0) {
    throw std::runtime_error("DamiaoCAN: can_bitrate must be greater than zero");
  }
  if (channel_ < 0 || channel_ > 1) {
    throw std::runtime_error("DamiaoCAN: can_channel must be 0 or 1");
  }

  const device_def_t device_type = channel_ == 1 ? DEV_USB2CANFD_DUAL : DEV_USB2CANFD;
  tools::logger()->info(
    "DamiaoCAN creating handle type={} for channel={}", device_type_name(device_type), channel_);
  handle_ = damiao_handle_create(device_type);
  if (!handle_) {
    throw std::runtime_error(
            std::string("DamiaoCAN: damiao_handle_create failed, type=") +
            device_type_name(device_type));
  }

  damiao_print_version(static_cast<damiao_handle *>(handle_));

  int device_cnt = damiao_handle_find_devices(static_cast<damiao_handle *>(handle_));
  if (device_cnt == 0) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    throw std::runtime_error(
            std::string("DamiaoCAN: no CAN device found, type=") + device_type_name(device_type));
  }

  device_handle * dev_list[16];
  int handle_cnt = 0;
  damiao_handle_get_devices(
    static_cast<damiao_handle *>(handle_), dev_list, &handle_cnt);
  if (handle_cnt == 0) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    throw std::runtime_error("DamiaoCAN: damiao_handle_get_devices failed");
  }

  tools::logger()->info("DamiaoCAN device list: detected {} device(s)", handle_cnt);
  for (int i = 0; i < handle_cnt; ++i) {
    char serial[256] = {0};
    device_get_serial_number(dev_list[i], serial, sizeof(serial));
    tools::logger()->info("DamiaoCAN device[{}] sn={}", i, serial);
  }

  int device_ordinal = 1;  // 1-based: 第1个设备
  bool has_explicit_ordinal = false;
  if (!selector.empty()) {
    const std::string prefix = "damiao:";
    if (selector.rfind(prefix, 0) == 0) {
      const auto body = trim_copy(selector.substr(prefix.size()));
      if (!body.empty() && is_all_digits(body)) {
        device_ordinal = parse_human_device_ordinal(body, "selector");
        has_explicit_ordinal = true;
      }
    }
  }

  std::string requested_sn = parse_selector_sn(selector);
  if (const char * env = std::getenv("DM_CAN_DEVICE_SN")) {
    const auto env_sn = trim_copy(env);
    if (!env_sn.empty()) {
      requested_sn = env_sn;
    }
  }

  if (const char * env = std::getenv("DM_CAN_DEVICE_INDEX")) {
    device_ordinal = parse_human_device_ordinal(env, "DM_CAN_DEVICE_INDEX");
    has_explicit_ordinal = true;
  }

  int device_index = -1;
  if (!requested_sn.empty()) {
    for (int i = 0; i < handle_cnt; ++i) {
      char serial[256] = {0};
      device_get_serial_number(dev_list[i], serial, sizeof(serial));
      if (requested_sn == serial) {
        device_index = i;
        break;
      }
    }

    if (device_index < 0) {
      tools::logger()->error(
        "DamiaoCAN: requested sn={} was not found in {} detected device(s)",
        requested_sn,
        handle_cnt);
      throw std::runtime_error("DamiaoCAN: no device matched DM_CAN_DEVICE_SN/selector sn=" + requested_sn);
    }
  } else {
    device_index = device_ordinal - 1;
    if (device_index < 0 || device_index >= handle_cnt || !dev_list[device_index]) {
      tools::logger()->error(
        "DamiaoCAN: device index out of range (requested ordinal={}, zero_based_index={}, detected={}). "
        "Set DM_CAN_DEVICE_INDEX=1 or can_interface=\"damiao:1\" when only device[0] is listed.",
        device_ordinal,
        device_index,
        handle_cnt);
      throw std::runtime_error(
              "DamiaoCAN: device index out of range (requested ordinal=" +
              std::to_string(device_ordinal) + ", detected=" + std::to_string(handle_cnt) + ")");
    }

    if (handle_cnt > 1 && !has_explicit_ordinal) {
      tools::logger()->warn(
        "DamiaoCAN: multiple devices found but no explicit SN/index set, fallback to device[0]. "
        "建议设置 DM_CAN_DEVICE_SN 或 selector=damiao:sn:<SN>");
    }
  }

  dev_ = dev_list[device_index];

  char selected_sn[256] = {0};
  device_get_serial_number(dev_list[device_index], selected_sn, sizeof(selected_sn));

  tools::logger()->info(
    "DamiaoCAN selecting device[{}] sn={} (ordinal={}, selector={}, env_index={})",
    device_index,
    selected_sn,
    device_ordinal,
    selector.empty() ? "<empty>" : selector,
    std::getenv("DM_CAN_DEVICE_INDEX") ? std::getenv("DM_CAN_DEVICE_INDEX") : "<unset>");

  tools::logger()->info("DamiaoCAN calling device_open...");
  if (!device_open(static_cast<device_handle *>(dev_))) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error("DamiaoCAN: device_open failed");
  }
  tools::logger()->info("DamiaoCAN device_open ok");

  tools::logger()->info(
    "DamiaoCAN calling device_channel_set_baud_with_sp(channel={}, bitrate={})...",
    channel_,
    can_bitrate);
  if (!device_channel_set_baud_with_sp(
        static_cast<device_handle *>(dev_),
        channel_,
        false,
        can_bitrate,
        can_bitrate,
        0.75f,
        0.75f)) {
    device_close(static_cast<device_handle *>(dev_));
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error(
            "DamiaoCAN: device_channel_set_baud_with_sp(channel=" + std::to_string(channel_) +
            ") failed");
  }
  tools::logger()->info("DamiaoCAN device_channel_set_baud_with_sp(channel={}) ok", channel_);

  tools::logger()->info("DamiaoCAN calling device_open_channel({})...", channel_);
  if (!device_open_channel(static_cast<device_handle *>(dev_), channel_)) {
    device_close(static_cast<device_handle *>(dev_));
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error(
            "DamiaoCAN: device_open_channel(" + std::to_string(channel_) + ") failed");
  }
  tools::logger()->info("DamiaoCAN device_open_channel({}) ok", channel_);

  s_rec_instance = this;
  device_hook_to_rec(static_cast<device_handle *>(dev_), rec_callback_static);
  device_hook_to_sent(static_cast<device_handle *>(dev_), sent_callback_static);
  device_hook_to_err(static_cast<device_handle *>(dev_), err_callback_static);

  opened_ = true;
  tools::logger()->info(
    "DamiaoCAN opened (DM-USB2FDCAN SDK), type={}, selected_sn={}, requested_sn={}, device_ordinal={}, device_index_zero_based={}, total_devices={}, channel={}, can_bitrate={}, debug={}",
    device_type_name(device_type),
    selected_sn,
    requested_sn.empty() ? "<empty>" : requested_sn,
    device_ordinal,
    device_index,
    handle_cnt,
    channel_,
    can_bitrate,
    debug_);
}

DamiaoCAN::~DamiaoCAN()
{
  if (s_rec_instance == this) s_rec_instance = nullptr;
  if (opened_ && dev_) {
    device_hook_to_rec(static_cast<device_handle *>(dev_), nullptr);
    device_hook_to_sent(static_cast<device_handle *>(dev_), nullptr);
    device_hook_to_err(static_cast<device_handle *>(dev_), nullptr);
    device_close_channel(static_cast<device_handle *>(dev_), channel_);
    device_close(static_cast<device_handle *>(dev_));
  }
  if (handle_) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
  }
  handle_ = nullptr;
  dev_ = nullptr;
  opened_ = false;
  tools::logger()->info("DamiaoCAN destructed.");
}

void DamiaoCAN::write(can_frame * frame) const
{
  if (!opened_ || !dev_) throw std::runtime_error("DamiaoCAN: device not open");

  uint8_t payload[8] = {0};
  const auto len = static_cast<std::uint8_t>(frame->can_dlc <= 8 ? frame->can_dlc : 8);
  std::memcpy(payload, frame->data, len);
  const bool ext = (frame->can_id & CAN_EFF_FLAG) != 0;
  const auto can_id = frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK);
  const auto tx_count = ++tx_request_count_;

  if (debug_) {
    const auto now = std::chrono::steady_clock::now();
    const bool log_this =
      tx_count <= 10 ||
      last_tx_debug_log_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration<double>(now - last_tx_debug_log_).count() >= 1.0;
    if (log_this) {
      last_tx_debug_log_ = now;
      tools::logger()->info(
        "[DamiaoCAN][TX_REQ {}] id=0x{:X} ch={} dlc={} ext={} data={} sent_cb={} rx_cb={} err_cb={}",
        tx_count,
        can_id,
        channel_,
        static_cast<int>(len),
        ext,
        format_payload(payload, len),
        tx_sent_count_.load(),
        rx_count_.load(),
        err_count_.load());
    }
  }

  device_channel_send_fast(
    static_cast<device_handle *>(const_cast<void *>(dev_)),
    channel_,
    can_id,
    1,
    ext,
    false,
    false,
    len,
    payload);
}

void DamiaoCAN::rec_callback_static(usb_rx_frame_t * rec_frame)
{
  if (s_rec_instance && rec_frame) s_rec_instance->on_rec(rec_frame);
}

void DamiaoCAN::sent_callback_static(usb_rx_frame_t * sent_frame)
{
  if (s_rec_instance && sent_frame) s_rec_instance->on_sent(sent_frame);
}

void DamiaoCAN::err_callback_static(usb_rx_frame_t * err_frame)
{
  if (s_rec_instance && err_frame) s_rec_instance->on_err(err_frame);
}

void DamiaoCAN::on_rec(usb_rx_frame_t * rec_frame)
{
  const auto count = ++rx_count_;
  if (debug_ && count <= 10) {
    tools::logger()->info("[DamiaoCAN][RX {}] {}", count, format_usb_frame(rec_frame));
  }

  can_frame kf;
  std::memset(&kf, 0, sizeof(kf));
  kf.can_id = rec_frame->head.can_id;
  kf.can_dlc = rec_frame->head.dlc <= 8 ? rec_frame->head.dlc : 8;
  std::memcpy(kf.data, rec_frame->payload, kf.can_dlc);

  if (rx_handler_) rx_handler_(kf);
}

void DamiaoCAN::on_sent(usb_rx_frame_t * sent_frame)
{
  const auto count = ++tx_sent_count_;
  if (debug_ && (count <= 20 || count % 100 == 0)) {
    tools::logger()->info("[DamiaoCAN][TX_SENT {}] {}", count, format_usb_frame(sent_frame));
  }
}

void DamiaoCAN::on_err(usb_rx_frame_t * err_frame)
{
  const auto count = ++err_count_;
  tools::logger()->warn("[DamiaoCAN][ERR {}] {}", count, format_usb_frame(err_frame));
}

}  // namespace io
