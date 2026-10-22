#include "io/can_bridge/aim_bridge_sender.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io::can_bridge
{
namespace
{

constexpr const char * kDefaultTopic = "/sp_vision/aim_can_frame";
constexpr int kDefaultQosDepth = 10;

void writeBigEndian16(std::array<std::uint8_t, 8> & data, const std::size_t offset, const std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  data.at(offset) = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  data.at(offset + 1U) = static_cast<std::uint8_t>(raw & 0xFFU);
}

}  // namespace

struct AimBridgeSender::Impl
{
  explicit Impl(const std::string & config_path)
  {
    const auto yaml = tools::load(config_path);
    topic = yaml["can_bridge_topic"] ? yaml["can_bridge_topic"].as<std::string>() : kDefaultTopic;
    const int qos_depth =
      yaml["can_bridge_qos_depth"] ? yaml["can_bridge_qos_depth"].as<int>() : kDefaultQosDepth;
    if (qos_depth <= 0) {
      throw std::runtime_error("can_bridge_qos_depth must be greater than zero");
    }
    can_send_hz = yaml["can_send_hz"] ? yaml["can_send_hz"].as<double>() : 0.0;

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_rclcpp_context = true;
    }

    node = std::make_shared<rclcpp::Node>("sp_vision_aim_bridge_sender");
    publisher = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth))));
    tools::logger()->info("[AimBridgeSender] publishing raw aim CAN payload to {}", topic);
  }

  ~Impl()
  {
    if (owns_rclcpp_context && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::string topic;
  double can_send_hz = 0.0;
  bool owns_rclcpp_context = false;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher;
  mutable std::chrono::steady_clock::time_point last_send_time{};
  mutable bool last_send_inited = false;
};

AimBridgeSender::AimBridgeSender(const std::string & config_path) : impl_(std::make_unique<Impl>(config_path)) {}

AimBridgeSender::~AimBridgeSender() = default;

void AimBridgeSender::send(const io::Command & command) const
{
  // 与原 CBoard::send 保持一致：can_send_hz > 0 时按配置频率限制 ROS 发布频率。
  if (impl_->can_send_hz > 0.0) {
    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / impl_->can_send_hz);
    if (impl_->last_send_inited) {
      const auto dt = now - impl_->last_send_time;
      if (dt < period) {
        return;
      }
    }
    impl_->last_send_time = now;
    impl_->last_send_inited = true;
  }

  const std::int16_t yaw_i = static_cast<std::int16_t>(command.yaw * 10);
  const std::int16_t pitch_i = static_cast<std::int16_t>(command.pitch * 10);
  const std::int16_t flag_i = command.control ? 1 : 0;
  const std::int16_t shoot_i = command.shoot ? 1 : 0;

  std::array<std::uint8_t, 8> payload{};
  writeBigEndian16(payload, 0U, yaw_i);
  writeBigEndian16(payload, 2U, pitch_i);
  writeBigEndian16(payload, 4U, flag_i);
  writeBigEndian16(payload, 6U, shoot_i);

  std_msgs::msg::UInt8MultiArray message;
  message.data.assign(payload.begin(), payload.end());
  impl_->publisher->publish(message);
}

}  // namespace io::can_bridge
