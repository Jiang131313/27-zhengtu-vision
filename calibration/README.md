# 相机-IMU 联合标定指南

## 快速开始

```bash
# 一键标定（录制 → 打包 → 标定）
bash calibration/calibrate.sh all

# 或分步执行
bash calibration/calibrate.sh record    # 录制数据集
bash calibration/calibrate.sh bag       # 生成 ROS bag
bash calibration/calibrate.sh calibrate # 运行 Kalibr 标定
```

## 相关文件

| 文件 | 用途 |
|------|------|
| `calibration/kalibr_config.yaml` | 标定 pipeline 参数（棋盘格尺寸、录制时长、帧率等） |
| `configs/demo.yaml` | 相机/IMU 参数（曝光、增益、内参、外参初值） |
| `calibration/calibrate.sh` | 标定流程脚本 |
| `calibration/calibrate.cpp` | 数据集录制程序 |
| `calibration/output/target.yaml` | Kalibr 标定板描述文件 |
| `calibration/output/camchain_prior.yaml` | 相机内参初值（给 Kalibr 的 priors） |
| `calibration/output/imu.yaml` | IMU 噪声参数 |
| `calibration/docker/kalibr_sb.patch` | Kalibr 棋盘格检测补丁 |

## 棋盘格

- 物理规格：**8×8 方格**，每格 **20mm**
- 内角点数：**7×7**（`kalibr_config.yaml` 中 `cols: 8, rows: 8` 表示方格数，内角点 = cols-1 × rows-1）
- 确保棋盘格在图像中**完全可见**，四周留白

## 相机参数

### 曝光时间

海康相机帧率硬编码为 **150 fps**（`io/hikrobot/hikrobot.cpp`），帧周期 ≈ 6.67ms，因此曝光时间不能超过此值。实际使用中相机帧率远达不到 150 fps，`SetFrameRate(150)` 仅为上限，**曝光设 20ms 可正常工作**。

```yaml
# configs/demo.yaml
exposure_ms: 20
gain: 20.0
```

如果图像过暗，优先增大增益而非曝光。

### 常见错误

- `exposure_ms > ~50` → 相机可能拒绝工作，日志出现 `MV_CC_GetImageBuffer failed: 0x80000007`
- `MV_CC_SetFloatValue("Gain", ...) failed: 0x80000102` → 此相机的 Gain 不接受浮点值，**这是一个 warning，不影响功能**，`hikrobot.cpp` 不可修改

## 棋盘格检测

### 预览 vs Kalibr 一致性

`calibrate.cpp` 预览窗口和 Kalibr 容器内的检测**使用完全相同的参数**，确保"预览能检测到，标定就一定能检测到"：

- 检测函数：`cv::findChessboardCornersSB`（优先）
- 标志位：`CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_EXHAUSTIVE`
- 回退方案：旧 `cv::findChessboardCorners`（仅 Kalibr 容器内）

### Kalibr 补丁

`calibration/docker/kalibr_sb.patch` 将 Kalibr 原生的棋盘格检测替换为 SB 检测器。修改后需要重建 Docker 镜像。

## Docker 镜像构建

### 正常构建（需要外网）

```bash
bash calibration/calibrate.sh docker-build
```

这会从 GitHub 克隆 Kalibr 源码，应用补丁，然后编译。

### 离线构建（无法访问外网）

当 GitHub/Docker Hub 不可达时，从已有镜像中提取源码并重建：

```bash
# 1. 从已有镜像提取 Kalibr 源码
docker create --name kalibr-tmp sp-vision-kalibr:cn
docker cp kalibr-tmp:/catkin_ws/src/kalibr /tmp/kalibr-src
docker rm kalibr-tmp

# 2. 修改源码（以更新棋盘格检测参数为例）
# 编辑 /tmp/kalibr-src/aslam_cv/aslam_cameras/src/GridCalibrationTargetCheckerboard.cpp

# 3. 创建 .git 目录（跳过 calibrate.sh 的 clone 检查）
mkdir -p /tmp/kalibr-src/.git

# 4. 在容器内编译并提交
docker run -d --name kalibr-fix sp-vision-kalibr:cn bash -c "while true; do sleep 60; done"
docker cp /tmp/kalibr-src/... kalibr-fix:/catkin_ws/src/kalibr/...
docker exec kalibr-fix bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin build -j\$(nproc)"
docker commit kalibr-fix sp-vision-kalibr:cn
docker rm -f kalibr-fix
```

## 数据集输出

每次录制前会自动清理 `calibration/output/dataset_*` 目录和文件，无需手动清理。

保留的文件（不会被删除）：
- `camchain_prior.yaml` — 相机内参先验
- `imu.yaml` — IMU 噪声参数
- `target.yaml` — 标定板描述

## 标定结果解读

标定完成后关注以下指标：

| 指标 | 优秀 | 可接受 | 差 |
|------|------|--------|-----|
| 重投影误差 (px) | < 0.5 | < 1.0 | > 1.5 |
| 陀螺仪误差 (rad/s) | < 0.001 | < 0.01 | > 0.05 |
| 加速度计误差 (m/s²) | < 0.001 | < 0.01 | > 0.1 |

**T_cam_imu 平移分量**：如果接近 0（如 < 1mm），说明采集时**平移激励不足**（只在原地旋转）。重新采集时加入前后左右平移运动。

## 常见问题

### 1. 预览窗口看不到棋盘格角点

- 确认棋盘格 8×8 方格，`kalibr_config.yaml` 中 `cols: 8, rows: 8`
- 确认棋盘格完全在图像内，不贴边
- 增大曝光/增益，确保图像足够亮
- 棋盘格需占图像至少 30% 面积

### 2. 相机无法获取图像

```
MV_CC_GetImageBuffer failed: 0x80000007
```

- 检查曝光时间是否 ≤ 50ms
- 物理拔插相机 USB 线（`libusb_reset_device` 不等于断电）
- 确认没有其他进程占用相机

### 3. Docker 构建失败（网络不通）

- 使用上述"离线构建"方法
- 基础镜像 `osrf/ros:noetic-desktop-full` 需提前缓存

### 4. 标定结果差

- 平移分量接近 0：采集时增加平移运动
- 重投影误差大：检查棋盘格是否平整，光照是否均匀
- 时间偏移跳变：IMU 配置是否一致（200Hz 输出）