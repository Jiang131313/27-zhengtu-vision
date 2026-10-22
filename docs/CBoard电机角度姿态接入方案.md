# CBoard 电机编码值姿态接入方案

## 1. 目标

新增一个姿态模块，读取电控通过 CAN 发送的 yaw、pitch 电机编码值，转换为四元数，并通过 `imu_at(timestamp)` 接口提供给哨兵主程序。

复用已有的 `io::CBoard` CAN 读取机制：不新开 CAN，新模块与 `CBoard` 共用同一个 CAN 后端，在 `CBoard::callback` 里按 canid 分流电机编码帧。

数据流：

```text
CBoard CAN 后端 (共用)
    -> CBoard::callback 按 canid 分流
    -> CBoardMotorPoseBuffer 解析 yaw/pitch 编码值
    -> 编码值转换为角度
    -> 角度转换为四元数
    -> imu_at(timestamp)
    -> src/sentry/main.cpp
```

## 2. 模块

新建文件：

```text
io/cboard_motor_pose_buffer.hpp
io/cboard_motor_pose_buffer.cpp
```

复用已有 `io::CBoard` 的 CAN 后端、`SocketCAN`/`DamiaoCAN` 和 `ThreadSafeQueue`。新模块只负责电机编码值的解析、转换、缓存与插值。

## 3. 模块接口

```cpp
namespace io
{
class CBoardMotorPoseBuffer
{
public:
  explicit CBoardMotorPoseBuffer();
  ~CBoardMotorPoseBuffer();

  // 由 CBoard 回调在收到 motor_angle_canid 帧时调用，写入编码值。
  void on_motor_angle(const can_frame & frame, steady_clock::time_point timestamp);

  Eigen::Quaterniond imu_at(
    std::chrono::steady_clock::time_point timestamp) const;

private:
  struct TimedMotorAngle
  {
    double yaw_rad;
    double pitch_rad;
    std::chrono::steady_clock::time_point timestamp;
  };

  std::deque<TimedMotorAngle> buffer_;
  mutable std::mutex mutex_;
};
}
```

编码值转角度所需的配置参数在构造时通过 `set_config` 传入，或由 `CBoard` 从同一份 YAML 读取后写入。

## 4. CAN 接收（复用 CBoard）

新模块不自己 open CAN。已有的 `io::CBoard::callback` 增加对 `motor_angle_canid` 的分流：

```cpp
// io/cboard.cpp CBoard::callback 内
if (frame.can_id == motor_angle_canid_) {
  motor_pose_buffer_->on_motor_angle(frame, timestamp);
  return;
}
```

- CBoard 保持现有的 quaternion 反馈（`quaternion_canid_`）和 `send()` 指令链路不变。
- 新模块通过 `on_motor_angle()` 接收电机编码帧，不与 CBoard 争用 CAN 后端。
- canid 错开：`quaternion_canid_`、`bullet_speed_canid_`、`send_canid_` 与新增 `motor_angle_canid` 互不相同。

## 5. 编码值转换

从配置读取：

```yaml
motor_yaw_zero: 0
motor_pitch_zero: 0
motor_yaw_scale: 0.0001
motor_pitch_scale: 0.0001
motor_yaw_direction: 1
motor_pitch_direction: 1
```

转换公式：

```cpp
const double yaw_rad =
  motor_yaw_direction *
  (yaw_encoder - motor_yaw_zero) *
  motor_yaw_scale;

const double pitch_rad =
  motor_pitch_direction *
  (pitch_encoder - motor_pitch_zero) *
  motor_pitch_scale;
```

`motor_yaw_scale` / `motor_pitch_scale` 是**编码值换算成弧度的比例系数**，单位 `rad/编码`。它综合编码器分辨率和减速比：`(编码值 − 零点)` 是无量纲的编码计数，乘上 scale 后得到弧度。

```text
scale = 2π / (一圈编码数 × 减速比)
```

例如编码器一圈 10000 计数、云台轴经减速比换算，scale 约 `0.0001 rad/编码`。具体按电控的编码器分辨率、减速比和协议单位填写。

`motor_yaw_scale` / `motor_pitch_scale` 负责量纲换算（编码 → 弧度），与零点（偏置）、方向（±1 翻转）配合完成：

```text
角度 = 方向 × (编码 − 零点) × 比例
```

`motor_yaw_zero` / `motor_pitch_zero` 零点由视觉侧自行标定确定（电控只提供原始编码值）。

标定方法：启动后读取电机反馈帧，取**接收到的第 50 帧**的 yaw/pitch 编码值作为零点写入配置。具体帧数可按情况确定——避开刚开始可能不准的帧，取云台进入稳定后的某一帧即可。

举例：

```cpp
// 收到第 N 帧（如 N=50）时，把当时的编码值记为零点
if (frame_count == 50) {
  motor_yaw_zero = yaw_encoder;
  motor_pitch_zero = pitch_encoder;
}
```

## 6. 四元数生成

复用 `io/dm_imu/dm_imu.cpp` 的姿态转换方式：

```cpp
Eigen::Quaterniond q =
  Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
  Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY());

q.normalize();
```

生成的四元数和时间戳写入姿态缓存。

## 7. `imu_at()` 实现

`imu_at(timestamp)` 按时间从缓存中获取 yaw/pitch：

```text
查询 timestamp 对应的前后两帧电机角度
    -> 对 yaw/pitch 做线性插值
    -> 插值后的角度转四元数
    -> 返回 Eigen::Quaterniond
```

缓存与插值结构参考 `io/imu_huaner/huaner_pose_buffer.cpp` 和 `io/imu_fxxk/hipnuc_pose_buffer.cpp`。

yaw 插值处理 `-pi/pi` 角度跳变。

## 8. `src/sentry/main.cpp` 接入

增加头文件：

```cpp
#include "io/cboard_motor_pose_buffer.hpp"
```

新模块通过 CBoard 共享同一 CAN，构造后注册到 CBoard：

```cpp
io::CBoard cboard(config_path);
io::CBoardMotorPoseBuffer motor_pose;          // 不 open CAN
cboard.set_motor_pose_buffer(&motor_pose);     // 让 CBoard 回调分流电机帧给新模块
```

姿态读取使用 `imu_at()`：

```cpp
q = motor_pose.imu_at(t + imu_time_offset);
```

后续姿态使用：

```cpp
solver.set_R_gimbal2world(q);
```

## 9. 配置参数

在 sentry 使用的 YAML 的 cboard 参数区中增加：

```yaml
motor_angle_canid: 0x102  # 读云台姿态（yaw/pitch 电机编码）的 canid，与自瞄 command 的 send_canid 分开

motor_yaw_zero: 0       # 零点由视觉侧标定（默认取接收的第 50 帧编码值，见第 5 节）
motor_pitch_zero: 0     # 零点由视觉侧标定（默认取接收的第 50 帧编码值，见第 5 节）
motor_yaw_scale: 0.0001
motor_pitch_scale: 0.0001
motor_yaw_direction: 1
motor_pitch_direction: 1
```

`motor_angle_canid`（读云台姿态）与已有的 `send_canid`（自瞄 command 下发）分开配置、互不相同，例如 `send_canid: 0x105`、`motor_angle_canid: 0x102`。

CAN 接口、波特率、通道等沿用 CBoard 已有的 `can_interface`、`can_bitrate`、`can_channel` 配置，不重复填写。

## 10. 电控协议

需要电控提供：

- 电机反馈 CAN ID；
- yaw 编码值位置；
- pitch 编码值位置；
- 数据长度；
- 数据类型；
- 字节序；
- 编码器分辨率；
- 减速比；
- yaw/pitch 正方向。

yaw/pitch 零点不在电控提供范围内，由视觉侧自行标定：默认取启动后接收到的第 50 帧编码值作为零点，帧数可按情况确定（见第 5 节）。

视觉侧统一使用弧度。

## 11. CMake

将新文件加入 `io/CMakeLists.txt`：

```cmake
add_library(io STATIC
    ...
    cboard_motor_pose_buffer.cpp
)
```

## 12. 最终结构

```text
io/cboard_motor_pose_buffer.cpp
  接收 CBoard 回调分流的电机编码帧
  转换为 yaw/pitch
  转换为四元数
  提供 imu_at(timestamp)

io/cboard.cpp
  callback 按 motor_angle_canid 分流电机帧给新模块
  quaternion 反馈与 send() 指令链路保持

src/sentry/main.cpp
  通过 imu_at(timestamp) 获取姿态
```
