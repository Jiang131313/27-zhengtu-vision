# RoboMaster 哨兵视觉自瞄系统

> 基于同济大学 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)（MIT License）二次开发的 RoboMaster 哨兵（Sentry）自动瞄准视觉系统。
>
> 本仓库只包含**视觉自瞄部分的源码**，不含构建产物、标定图片、运行日志和厂商 SDK 二进制文件。

---

## 1. 项目定位

哨兵机器人全自动运行，没有操作手，云台的目标搜索、跟踪、弹道解算和开火全部由视觉负责。本项目实现了完整的视觉自瞄链路：

```
海康 MV-CS016-10UC (1440×1080)
        │
        ▼
   装甲板检测  YOLOv5(OpenVINO) + 传统灯条检测兜底
        │
        ▼
   位姿解算    PnP + yaw 多解搜索（solver）
        │
        ▼
   目标估计    EKF 整车模型（4 装甲板 / 平衡步兵 2 块 / 前哨站 3 块）
        │
        ▼
   瞄准决策    提前量预测 + 弹道下坠迭代解算 + 瞄准点选择
        │
        ▼
   云台姿态    电控回传 yaw/pitch 电机编码器 → 四元数 → R_gimbal2world
        │
        ▼
   火控 + 下发  三条件开火门 → CAN 0x105（增量角）→ 电控
```

关键技术点：

- **无 ROS 依赖**：新成员不需要学 ROS 就能上手；只为哨兵的导航通信预留了 ROS 接口。
- **模块化 + 独立测试程序**：识别、解算、估计、决策、通信每一层都有单独的 `tests/` 入口，可以脱离整车单测。
- **姿态来源可切换**：IMU（`imu_huaner` / `imu_fxxk`）与电控电机编码器（本仓库重点改造）二选一，通过配置切换。

---

## 2. 来源说明与分工

| 部分 | 来源 |
| --- | --- |
| 整体框架、检测网络、PnP 解算、EKF 整车模型、轨迹规划器、标定工具链 | 同济大学 TongjiSuperPower `sp_vision_25`（MIT） |
| 姿态来源改造（电机编码器接入）、CAN 通信层重构、标定流程落地、EKF 门限修正、瞄点下发链路、火控逻辑、灰板跟踪、调试可视化体系 | 本人 |

原始项目的 `LICENSE`（MIT, Copyright © 2025 TongjiSuperPower）**原样保留**，未做任何修改。原项目 README 存档于 [`docs/原队伍开源说明.md`](docs/原队伍开源说明.md)。

> 说明：`src/sentry/command_builder.*`、`src/sentry/target_filter.*`、`io/cboard_motor_pose_buffer.*`、`src/sentry/debug_overlay.*` 等文件为同济上游**不存在**的本地新增模块，属于本人的改造范围。

---

## 3. 我做了什么

详细逐条说明（问题现象 → 根因 → 改法 → 效果）见 **[`docs/改进说明.md`](docs/改进说明.md)**。这里只列摘要。

### 3.1 姿态来源：从 IMU 换成电控电机编码器（"电机补偿"部分）

**状态：已完成并实机验证。**

按原设计，云台姿态由 C 板 BMI088 IMU 提供；但实测 IMU 不可用（`configs/demo.yaml` 里 `imu_enabled: false`），代码只能退化成单位四元数，等于认为云台一直没转过。改造后姿态直接取自电控回传的 yaw / pitch 电机编码器：

```
电控 CAN 0x106 ──► CBoard::callback 按 canid 分流
                ──► CBoardMotorPoseBuffer（解析 → 角度 → 四元数 → 时间戳缓存）
                ──► motor_pose.imu_at(t + imu_time_offset)
                ──► solver.set_R_gimbal2world(q)
```

- **为什么让它发编码值，而不是直接发欧拉角**：电控端 `GimbalAimReceive_Task` 处理的是增量角 `d = 目标 − 当前`，它自己不维护世界系绝对姿态；但它底层闭环控制的就是 yaw / pitch 两个电机，编码器读数是最原始、最可信的姿态源。而且 `0x106` 本来就在周期性发送，只需要在数据区里带上两个编码值，不用新开帧、不增加总线负载。
- 新增 `io/cboard_motor_pose_buffer.{hpp,cpp}`，复用 `io::CBoard` 已有的 CAN 后端，不新开设备、不与指令下发争用总线。
- 编码值 → 角度的换算完全配置化：`motor_yaw_scale` / `motor_pitch_scale` / `motor_yaw_zero` / `motor_pitch_zero` / `motor_yaw_direction` / `motor_pitch_direction`。
  - 实测标定结果：yaw 为 8192 线编码器，`scale = 2π/8192 = 0.00076699 rad/count`（0.0439°/count）；pitch 为整度输出，`scale = π/180`，并实测确认电控 pitch 正方向与云台系 Ry 相反，故 `motor_pitch_direction = -1`。
- 零点自动标定：启动后第 50 帧的编码值作为零点，避开开机异常帧；并对 yaw 零点加了 `[0, 8192)` 范围检查（曾因异常帧把零点标歪，导致 yaw 偏 1100°、云台失控）。
- `imu_at()` 按时间戳在缓存里取前后两帧做线性插值，yaw 插值处理 ±π 跳变。

设计文档见 [`docs/CBoard电机角度姿态接入方案.md`](docs/CBoard电机角度姿态接入方案.md)。

**已知限制**（写在这里是为了不让它看起来比实际更完整）：

- 只有 yaw / pitch 两轴，**没有 roll**（四元数由 `AngleAxisd(yaw, Z) * AngleAxisd(pitch, Y)` 构造）。底盘不动时够用，底盘剧烈运动时世界系会偏。
- pitch 分辨率被电控量化到 **1°/count**，姿态精度上限约 ±0.5°。
- 姿态时间戳取 CAN 帧到达时刻，不是电控的产生时刻，存在总线传输延迟。
- 缓存为空时静默返回单位四元数（不告警），如果 CAN 断了程序不会崩溃但会瞄不准。

### 3.2 CAN 通信层重构

原始实现的指令下发挂在图像帧率上（约 100 Hz，且随检测耗时抖动），云台响应慢且抖。改造后：

- **发送解耦到独立线程**，固定频率发最新一帧指令（`can_send_hz`，默认 200 Hz），彻底消除"帧率绑架发送频率"。
- 下发语义对齐电控：改为发**增量角**（`d = 目标 − 当前`），与电控 `GimbalAimReceive_Task` 的 `d_yaw` / `d_pitch` 语义一致（原来发绝对角，云台会朝单方向跑飞）。
- `control = 0` 时发 0 增量而不是把云台拉回零点（原来目标丢失时云台会被猛甩）。
- 增量限幅（`send_yaw_clamp` / `send_pitch_clamp`，pitch 取 +40/−20），修目标跳变时的猛甩。
- 方向 / 增益 / 偏置全部做成配置（`send_*_sign` / `send_*_gain` / `send_*_bias` / `send_scale`），现场调参不用重编译。
- 修正 `can_interface` 为实际插入的达妙 USB2FDCAN 序列号（原值是另一台机器的）。
- `bullet_speed_canid` 由 `0x101` 改为 `0x7FF`，避开电控遥测帧导致的 `Invalid feedback` 刷屏。

### 3.3 相机标定与手眼标定

- 移植了 Jetson 版 `handeye_calib` 手眼标定工具，检测放到独立线程，主线程不再被占满导致窗口无响应；加入半分辨率优先 + 失败回退全分辨率 + 保底缓存，检测帧率与稳定性都上来了。
- 检测按 `1.0 → 0.7 → 0.4` 的优先级降级，修角点精度不足导致的姿态乱跳。
- 标定板参数对齐物理板（7×7 内角点 / 20 mm）。
- 新内参合并进 `configs/demo.yaml`（`cy` 与原值差约 60 px，PnP 与手眼标定都必须用新值）。
- 手眼标定定稿：`t_camera2gimbal = 21 cm`（两组独立数据互印证），并由实测确认 pitch 符号相反。

### 3.4 EKF 卡方门限数量级修正

`tools/extended_kalman_filter.cpp` 里 `nis_threshold` / `nees_threshold` 原来都是 **0.711**——那是 χ²(4) 的**下尾 5% 分位**（P(χ²(4) < 0.711) ≈ 0.05），查表时把 α = 0.95 那一栏当成了"置信度 95%"。本工程一次更新 4 维测量，NIS 服从 χ²(4)、**均值本身就是 4**，95% 分位是 **9.488**；NEES 用 11 维全状态协方差，χ²(11) 的 95% 分位是 **19.675**。而判据是 `nis > threshold` 即算失败，所以原值等于只放行 5% 的测量，剩下 95% 的正常帧全被判离群，表现为"跟踪不灵敏、一顿一顿"。已改为 `9.488` / `19.675`。

### 3.5 瞄点下发链路（`src/sentry/command_builder.cpp`）

新增一层"瞄准指令构建"，原先的实现有两个问题：

1. **弹道解算结果被丢弃**：Aimer 里 10 次迭代算出的提前量、下坠补偿、`yaw_offset` / `pitch_offset` 全部被覆盖，下发的只有当前帧的检测角。5 m 距离上 11 m/s 弹速的下坠补偿约 **11.9°（≈50 cm）**，等于完全不打提前量和下坠。
2. **每帧必须匹配到装甲板，否则下发 `control = 0`**：跟踪在 `temp_lost` 缓冲期时云台直接冻结，目标随即离开视野，本该靠缓冲救回来的目标必然进入 `lost`。

改造后下发的是 Aimer 世界系瞄准方向，经 `R_gimbal2world^T` 转到枪管系再转角度，**提前量、下坠、两个偏移全部生效**；同时去掉了"本帧必须匹配到装甲板"的硬门槛。

### 3.6 火控（`tasks/auto_aim/shooter.cpp`）

原实现非小陀螺时直接 `fire_ready = true`——只要判定有目标就每帧开火，射击散布差、弹药浪费。改为同济上游的三条件门：

| 条件 | 含义 |
| --- | --- |
| `|last_command.yaw − command.yaw| < tolerance × 2` | 指令没有突变（排掉装甲板切换瞬间） |
| `|command.yaw| < tolerance` | 云台真的跟上了上一次指令（枪管到位） |
| `aimer.debug_aim_point.valid` | 瞄点有效（弹道可解） |

三者同时成立才开火，实测射击一致性明显提升。

### 3.7 灰板（被打灭的装甲板）跟踪

装甲板被击中后会熄灭变灰。原实现有两处按颜色硬过滤，把 `extinguish` 的装甲板全部丢掉，导致"打中一块之后目标消失、重新进入搜索"。

`tasks/auto_aim/tracker.cpp` 改为：非 `lost` 状态时放行灰板进入 EKF（`lost` 时仍不放行，避免拿分不清敌我的灰板重新起锁）。EKF 的 `update_target` 只按 `name + type` 匹配、不看颜色，所以放行后灰板会正常续上原有轨迹。

### 3.8 调试可视化

单相机、无操作手的哨兵最难的地方是"看不见它在想什么"。补了一整套：

- 调试窗口只显示**真正下发给电控的**增量角与 CAN 原始 int16 字节（原先显示的是绝对目标角 + 现算的假 raw，误导排查）。
- 1 Hz 打印 `[TX->EC]`（我方下发）/ `[RX]`（电控回传）/ 差值 / 瞄准误差。
- 1 Hz 打印 `control` 被关掉的 5 种原因，直接定位"一顿一顿"是哪个环节把指令关了。
- 1 Hz 打印 EKF 的 NIS 失败率，用于确认目标被反复丢弃是不是卡方门限导致。

### 3.9 稳定性与部署

- 修 `Target` 装甲板匹配的越界读：硬编码 3 改为 `min(3, armor_num_)`，平衡步兵（2 块）和单块调试（1 块）时会崩。
- `diverged()` 发散误判三轮收敛（详见改进说明），修"自瞄启动几秒后云台停转"。
- 加 `autostart` watchdog 看门狗脚本。
- 加 `single_armor_debug` 开关，单块装甲板调试时按 1 块建模并跳过半径检查。

---

## 4. 目录结构

```
src/sentry/           哨兵主程序：main / command_builder / target_filter / debug_overlay / config
tasks/auto_aim/       自瞄算法：detector / yolo / solver / target / tracker / aimer / shooter / planner / voter
tasks/omniperception/ 全向感知决策（多相机，本机只有单相机，未启用）
io/                   硬件抽象：camera / hikrobot / mindvision / usbcamera
                      cboard(电控CAN) / cboard_motor_pose_buffer(电机编码姿态) / damiao_can / socketcan
                      imu_huaner / imu_fxxk(IMU) / gimbal / serial / ros2
tools/                数学与基础库：EKF / 轨迹解算 / 坐标变换 / 日志 / 录制
tests/                各模块独立测试程序（相机、识别、解算、通信、弹道…）
configs/              配置文件（demo / standard3 / standard4 等）
calibration/          相机内参与手眼标定工具
assets/               检测模型（yolov5 IR + 分类网络）与标定参数
docs/                 设计文档、改进说明、调试记录、原队伍开源说明
deploy/               开机自启项（sp_vision.desktop）
```

---

## 5. 构建与运行

### 环境

- Ubuntu 22.04，C++17，CMake ≥ 3.16
- OpenVINO 2024+（推理后端）
- Ceres Solver（PnP / 优化）
- Eigen3、OpenCV 4.5
- 相机 SDK：海康 MVS（`MV-CS016-10UC`）或迈德威视 MindVision
- CAN：达妙 DM-USB2FDCAN（`34b7:6877`）
- ROS Humble 仅在需要导航通信时启用，**自瞄本身不依赖 ROS**

### 厂商 SDK 需要自行准备

出于体积与授权考虑，仓库内**不包含**厂商二进制库：

```
io/hikrobot/lib/amd64/libMvCameraControl.so
io/mindvision/lib/amd64/libMVSDK.so
io/damiao/lib/linux/libdm_device.so
```

请自行安装对应 SDK，并把 `.so` 放回上述路径（或让 `LD_LIBRARY_PATH` 指向 SDK 安装目录）。

### 编译

```bash
# 海康相机需要先加载 MVS 环境变量，否则运行时枚举不到设备
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export GENICAM_GENTL64_PATH=/opt/MVS/lib/64
export ALLUSERSPROFILE=/opt/MVS/MVFG

mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 运行

```bash
mkdir -p logs records      # logger 不会自动建目录，缺 logs/ 时首次写日志会抛异常
./build/sentry configs/demo.yaml
```

只调相机和识别、不接电控时，可以用独立测试程序（完全不碰 CAN）：

```bash
./build/camera_detect_test configs/demo.yaml      # 相机 + YOLO
./build/camera_detect_test configs/demo.yaml -t   # 走传统检测
```

### 关键配置（`configs/demo.yaml`）

| 键 | 说明 |
| --- | --- |
| `enemy_color` | 敌方颜色（`red` / `blue`） |
| `priority_target` | 优先级目标（`sentry` / `outpost` / `base` …） |
| `display` | 是否开调试窗口（需要可用的 X11 / DISPLAY） |
| `can_disabled` | `true` 时完全不碰 CAN，用单位四元数跑（无电控时调试用） |
| `motor_angle_canid` | 电控回传云台姿态的 CAN ID（`0x106`） |
| `send_canid` | 自瞄指令下发 CAN ID（`0x105`） |
| `can_send_hz` | 指令下发频率（`200`） |
| `motor_yaw_scale` / `motor_pitch_scale` | 编码值 → 弧度的比例 |
| `motor_pitch_direction` | pitch 方向（实测 `-1`） |
| `t_camera2gimbal` / `t_muzzle2gimbal` | 相机、枪口相对云台的外参 |
| `yaw_offset` / `pitch_offset` | 瞄准静态偏置 |
| `first_tolerance` / `second_tolerance` | 开火门限（度） |

---

## 6. 日志与版本管理

### 运行日志

每次启动在**当前工作目录**下新建一个日志文件，文件名即启动时刻：

```
logs/2026-10-22_23-31-15.log
```

- 由 `tools/logger.cpp:17` 的 `fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", ...)` 生成，秒级精度；
- 同时写文件和控制台（彩色），两个 sink 的级别都固定为 `debug`；
- `flush_on(spdlog::level::info)` ⇒ 崩溃时最多丢掉最后几条 `debug`，`info` 及以上已落盘；
- **路径是相对路径**，必须在项目根目录启动，否则 `logs/` 会建在别处；
- `basic_file_sink_mt` 不自动创建目录，所以首次运行前要 `mkdir -p logs`，否则第一次写日志直接抛 `spdlog_ex`。

### 其他运行产物

| 路径 | 内容 | 生成位置 |
| --- | --- | --- |
| `logs/<时间戳>.log` | 完整运行日志（info + debug） | `tools/logger.cpp:17` |
| `records/<时间戳>.avi` + `.txt` | 录制视频与逐帧姿态/时间戳文本 | `tools/recorder.cpp:18-23`（目录自动创建） |
| `imgs/` | 检测结果出图，仅显式调用 `save()` 时写 | `tasks/auto_aim/yolos/yolov5.cpp:32` |
| `MvSdkLog/` | 海康 MVS SDK 自身日志 | 相机 SDK |

一键清理：`rm -rf logs records imgs MvSdkLog`。

> **和「按运行编号分目录」的项目的区别**：本项目日志按**启动时刻**命名，一次启动一个文件，没有 `log/runs/<run_id>/` 这种目录层级，也没有用环境变量指定日志根目录。原因是哨兵是单机常驻进程，一次上电即一次会话，时间戳足够唯一定位；需要改落盘位置时，改 `tools/logger.cpp:17` 的格式化字符串即可。

### 版本管理

- 主分支 `main`。
- `.gitignore` 排除：**构建产物**（`build*/`、`cmake-build-*/`、`CMakeCache.txt`、`*.o`、`*.a`）、**运行产物**（`logs/`、`records/`、`imgs/`、`patterns/`、`MvSdkLog/`、`Camera/`）、**标定输出**（`calibration/output/` 及各 workspace、`*.bag`/`*.db3`/`*.mcap`）、**视频素材**、**厂商 SDK 二进制**（`io/**/lib/**/*.so`）。
- 模型权重只保留 demo 跑得起来的三份：`assets/yolov5.bin` + `assets/yolov5.xml`（检测）与 `assets/tiny_resnet.onnx`（装甲板类型分类）；`assets/yolo11*`、`assets/yolov8*` 需另行下载。
- 因此 **clone 后不能直接编译**，缺海康 / 迈德威视 / 达妙的 `.so`（上一节已说明）。这是有意的：厂商 SDK 各有授权，不能跟随 MIT 一并分发。
- 本仓库是发布用的**干净副本**，不含构建产物、标定图片和二进制历史。开发在本地工作树进行，改完同步到本目录再提交：

```bash
git add -A
git commit -m "改了什么"
git push
```

---

## 7. 已知限制与后续方向

- **姿态无 roll**：单靠 yaw/pitch 两轴编码器无法还原滚转，底盘大机动时世界系会偏。车上本来就有 C 板 BMI088 IMU，把它接回来与编码器做融合即可（方案见 `docs/双Yaw轴姿态融合方案.md`）。
- **EKF 半径状态的过程噪声为 0**：`Target` 的 `Q` 矩阵第 8/9/10 行（`r` / `l` / `h`）全为 0，半径协方差只减不增，一旦被推偏就永久冻结。这是 `diverged()` 误判的根因，目前用阈值放宽规避，没有根治。
- **弹速来自电控 CAN，协议仍在与电控对齐**：视觉侧对量纲（×1 / ×10 / ×100）做了兼容，存在误接受垃圾值的风险。
- **单相机无全向感知**：同济原版靠三相机（后 + 左 + 右）在 `lost` 状态下做全向搜索，本机只有单相机，丢锁后的重捕获能力受限。
- **只有云台编码器、无底盘里程计**：底盘平移时世界系不跟车走，EKF 预测会偏；原地或微动瞄准不受影响。

---

## 8. 许可

本项目遵循 MIT License，见 [`LICENSE`](LICENSE)。

原始代码版权：Copyright © 2025 TongjiSuperPower。

---

## 9. 致谢

- [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) —— 整体框架、检测网络、EKF 与规划器算法基础
- RoboMaster 社区各队开源的检测模型与标定思路
