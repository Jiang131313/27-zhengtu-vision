#ifndef IO__CAN_BRIDGE__AIM_BRIDGE_SENDER_HPP
#define IO__CAN_BRIDGE__AIM_BRIDGE_SENDER_HPP

#include <memory>
#include <string>

#include "io/command.hpp"

namespace io::can_bridge
{

class AimBridgeSender
{
public:
  explicit AimBridgeSender(const std::string & config_path);
  ~AimBridgeSender();

  AimBridgeSender(const AimBridgeSender &) = delete;
  AimBridgeSender & operator=(const AimBridgeSender &) = delete;

  void send(const io::Command & command) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace io::can_bridge

#endif  // IO__CAN_BRIDGE__AIM_BRIDGE_SENDER_HPP
