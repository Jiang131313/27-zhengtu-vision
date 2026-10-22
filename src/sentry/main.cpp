#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/cboard_motor_pose_buffer.hpp"
#include "io/imu_huaner/huaner_pose_buffer.hpp"
#include "src/sentry/command_builder.hpp"
#include "src/sentry/config.hpp"
#include "src/sentry/debug_overlay.hpp"
#include "src/sentry/target_filter.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 程序只需要一个配置文件路径；没给路径时直接打印帮助，别让后面的模块拿空路径启动。
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 退出控制和配置先准备好，后面的相机、串口、自瞄模块都从同一份 yaml 里读参数。
  tools::Exiter exiter;
  const auto config = sentry_app::load_config(config_path);

  // CBoard 管 CAN 指令/反馈，相机管图像输入，维特/幻尔 IMU 可按配置提供姿态。
  io::CBoard cboard(config_path);
  // 电机编码姿态：电控通过 CAN 发 yaw 编码 + pitch 角度，这里转成四元数当姿态用。
  // 不新开 CAN，注册到 CBoard 后由它回调分流电机帧。
  io::CBoardMotorPoseBuffer motor_pose;
  cboard.set_motor_pose_buffer(&motor_pose);
  io::Camera camera(config_path);
  std::unique_ptr<io::HuanerPoseBuffer> imu;
  if (config.imu_enabled) {
    imu = std::make_unique<io::HuanerPoseBuffer>(config.imu_port, config.imu_baudrate);
    tools::logger()->info(
      "[Sentry] Huaner/WIT IMU opened: {} @ {}", config.imu_port, config.imu_baudrate);
  } else {
    tools::logger()->warn("[Sentry] Huaner/WIT IMU disabled by config; using identity pose.");
  }

  // 自瞄主链路按“检测 -> 解算 -> 跟踪 -> 瞄准 -> 火控”串起来。
  // 开显示时 YOLO 会额外开 detection 窗口，方便现场确认模型有没有正常框到装甲板。
  auto_aim::YOLO detector(config_path, config.show_display);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  const auto imu_time_offset =
    duration_cast<steady_clock::duration>(duration<double>(config.imu_time_offset_s));

  // 记录当前模式，模式变化时只打一条日志，调车时能看清什么时候切了状态。
  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  int frame_count = 0;

  while (!exiter.exit()) {
    ++frame_count;

    // 一帧开始时先对齐三件事：图像、姿态、当前工作模式。
    camera.read(img, t);
    // Kalibr 给的是 t_imu = t_cam + offset；用图像时间加偏移去查对应 IMU 姿态。
    // 姿态优先级：电控电机编码（motor_angle_canid）> 串口 IMU > 单位四元数
    if (cboard.has_motor_pose()) {
      q = motor_pose.imu_at(t + imu_time_offset);
    } else if (imu) {
      q = imu->imu_at(t + imu_time_offset);
    } else {
      q = Eigen::Quaterniond::Identity();
    }
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // Solver 只保存姿态关系；后面的预测、开火判断都基于这份云台到世界坐标的旋转。
    solver.set_R_gimbal2world(q);
    const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    const Eigen::Vector3d shoot_origin_world = solver.R_gimbal2world() * config.t_muzzle2gimbal;

    // 检测先给出候选装甲板，再按敌方颜色和优先目标做过滤，后面的模块只处理已经筛好的结果。
    auto armors = detector.detect(img, frame_count);
    sentry_app::filter_priority_target(armors, config.enemy_color, config.priority_target);

    // 进入 Tracker 前的装甲板数快照：Tracker 会改写 armors（消耗掉已分配的目标），
    // 必须在 track() 之前记录，否则 before/after 会恒等，调试面板失去意义。
    const std::size_t armors_before_tracker = armors.size();

    // Tracker 维护跨帧目标状态；它可能会改写 armors。
    const auto targets = tracker.track(armors, t);

    // Aimer 根据跟踪目标算瞄准量，command_builder 再把瞄准结果翻译成下发给下位机的指令。
    const auto aimer_command = aimer.aim(targets, t, cboard.bullet_speed, shoot_origin_world);
    auto command =
      sentry_app::build_auto_aim_command(aimer_command, targets, armors, aimer, solver, config);

    // 火控 shoot：control=1 且存在目标时，由 Shooter 判定是否开火。
    if (command.control && !targets.empty()) {
      command.shoot = shooter.shoot(command, aimer, targets, ypr);
    } else {
      shooter.reset_debug_state();
      command.shoot = false;
    }

    cboard.send(command); // 发送给电控

    if (config.show_display) {
      // 显示层只拿 YOLO 自己缓存的绘制结果画绿框，不反过来影响控制链路的数据。
      const std::size_t armors_after_tracker = armors.size();
      const auto draw_armors = detector.get_last_drawn_armors();
      const sentry_app::DebugOverlayInput debug_input{
        frame_count,
        cboard.bullet_speed,
        mode,
        armors_before_tracker,
        armors_after_tracker,
        command,
        draw_armors,
        targets,
        tracker,
        aimer,
        shooter};

      if (!sentry_app::show_debug_overlay(img, solver, debug_input)) break;
    }
  }

  return 0;
}

/*
    make -C build -j$(nproc)
    ./build/sentry configs/demo.yaml -d
*/
