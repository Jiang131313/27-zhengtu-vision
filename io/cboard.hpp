#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <memory>

#include "io/can_backend.hpp"
#include "io/command.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// 电机编码姿态缓冲（由 CBoard 回调分流电机帧给它）
class CBoardMotorPoseBuffer;

enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

  // 注册电机姿态缓冲：CBoard 回调会把 motor_angle_canid 的帧分流给它
  void set_motor_pose_buffer(CBoardMotorPoseBuffer * buffer);

  // 是否已配置电机姿态来源（motor_angle_canid != 0 且已注册 buffer）
  bool has_motor_pose() const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  std::unique_ptr<ICan> can_;
  IMUData data_ahead_;
  IMUData data_behind_;
  bool can_disabled_ = false;
  bool can_send_only_ = false;  // 仅发送、不等待电控反馈（单向通信）
  bool can_read_imu_ = true;
  double can_send_hz_ = 0.0;    // CAN 发送频率（Hz），0 表示按调用频率发送
  int can_bitrate_ = 1000000;   // 达妙直连经典 CAN 波特率
  int can_channel_ = 0;         // 达妙 USB-CAN 通道，单通道通常为 0，双通道可用 1

  int quaternion_canid_, bullet_speed_canid_, send_canid_, motor_angle_canid_ = 0;
  CBoardMotorPoseBuffer * motor_pose_buffer_ = nullptr;
  std::string config_path_;  // 保存配置路径：set_motor_pose_buffer() 之后要用它读电机编码参数
  mutable std::chrono::steady_clock::time_point last_send_time_{};
  mutable bool last_send_inited_ = false;

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);

  // 把 yaml 里的电机编码参数（zero/scale/direction）喂给 motor_pose_buffer_
  void apply_motor_pose_config();
};

}  // namespace io

#endif  // IO__CBOARD_HPP
