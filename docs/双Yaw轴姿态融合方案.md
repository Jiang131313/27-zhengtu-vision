# 双Yaw轴姿态融合方案

## 1. 本文范围

这版只讨论实际启动链路：

- `fuck.sh`
- `./build/sentry`
- `configs/demo.yaml`

目标只有一个：

**尽量复用现有 IMU 和视觉代码，只在姿态入口补最小一层，让双 yaw 轴能接入现有 `standard` 主链路。**

本文不展开 `standard_mpc`、`mt_standard`、`dm_imu` 单独链路，也不改 `Tracker / Aimer / Shooter` 主体设计。

---

## 2. 当前真实链路

`fuck.sh` 最后实际运行的是：

```bash
./build/sentry configs/demo.yaml -d
```

`src/sentry/main.cpp` 里的主链路可以缩成下面这几步：

1. `camera.read(img, t)`
2. `q = cboard.imu_at(t - 1ms)`
3. `solver.set_R_gimbal2world(q)`
4. `detector.detect(...)`
5. `tracker.track(...)`
6. `aimer.aim(...)`
7. `shooter.shoot(...)`
8. `cboard.send(command)`

结论很直接：

- 现有视觉链已经是完整的
- 姿态入口只有一处，就是 `solver.set_R_gimbal2world(...)`
- 所以双 yaw 轴要最小改动落地，应该只在这一步前面补融合

不要先去改 `Tracker`、`Aimer`、`Shooter`，那样改动面会很大，而且不是根因位置。

---

## 3. 当前代码的关键假设

`tasks/auto_aim/solver.cpp` 里现在的姿态更新是：

```cpp
Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
```

这等价于一个假设：

**`gimbal` 和 `imu body` 之间只有固定外参，没有动态关节。**

这个假设只在下面情况成立：

- IMU 已经跟着最终枪管姿态一起动
- 或者说 IMU 看到的 yaw，本身就已经是最终解算要用的 yaw

如果你的结构是“双 yaw”：

- 一级 yaw 在 IMU 所在刚体上
- 二级 yaw 在枪管本体上
- IMU 看不到这个二级 yaw

那现在的固定外参假设就不成立，必须把二级 yaw 电机角度补进来。

---

## 4. 先说明一个现实问题

当前 `configs/demo.yaml` 里有三个配置是：

- `aim_mode: "delta"`
- `can_send_only: true`
- `use_gimbal_feedback: false`

这意味着当前 `fuck.sh` 默认跑的是“尽量少依赖反馈”的链路。

尤其是 `can_send_only: true` 时，`io::CBoard::imu_at()` 会直接返回单位四元数，不会拿真实反馈四元数。

所以要做双 yaw 姿态融合，**前提不是先大改视觉，而是先让这条链真正拿到 IMU 反馈。**

如果还保持 `can_send_only: true`，那双 yaw 融合在 `standard` 这条链上基本没有意义。

---

## 5. 最小改动方案

### 5.1 方案原则

保留下面这些现有模块不动：

- `io::Camera`
- `auto_aim::YOLO`
- `auto_aim::Solver`
- `auto_aim::Tracker`
- `auto_aim::Aimer`
- `auto_aim::Shooter`
- `io::CBoard` 的现有收发框架

只补一层很薄的“姿态融合”：

- 输入：`q_imu`
- 输入：电控直接给的二级 yaw 标量角度 `theta_yaw2`
- 可选输入：二级 yaw 角速度 `omega_yaw2`
- 输出：最终给 `Solver` 用的枪管姿态

这里明确一下：

- `q_imu` 仍然表示 IMU 的一级姿态四元数
- 二级 yaw 这一路**不需要四元数**
- 电控如果能直接给一个角度值，那就是最适合接入这一层的输入

也就是说，双 yaw 第一版推荐的数据形态不是：

- `q_imu`
- `q_yaw2`

而是：

- `q_imu`
- `theta_yaw2`

### 5.2 建议插入位置

就放在 `src/sentry/main.cpp` 这一段前后：

```cpp
q = cboard.imu_at(t - 1ms);
solver.set_R_gimbal2world(q);
```

目标改成这种结构：

```cpp
q_imu = cboard.imu_at(t - 1ms);
theta_yaw2 = ...;  // 电控直接给的二级 yaw 角度
R_fused = fuse_dual_yaw_pose(q_imu, theta_yaw2);
solver.set_R_gimbal2world(R_fused);
```

这里不要求一定是这个函数名，意思是：

- 视觉后半段不动
- 只把“送给 `Solver` 的姿态”换成融合后的姿态

### 5.3 核心思想

把原来的：

- 固定 `R_gimbal2imubody`

升级成：

- 固定外参
- 加一个二级 yaw 的动态旋转

也就是：

```text
R_gimbal2world
= R_imu2world(q_imu)
 * R_static_1
 * Rz(theta_yaw2)
 * R_static_2
```

这里最重要的不是公式左右乘的字面形式，而是这个原则：

**原先的常量外参，要变成“常量外参 + 二级 yaw 动态项”。**

具体正负号、旋转方向、左右乘顺序，最后按你们坐标系定义和台架测试定。

再强调一次：

- 这个动态项直接用角度 `theta_yaw2` 进 `Rz(theta_yaw2)` 就行
- 不需要让电控把二级 yaw 再包装成一个四元数回传

### 5.4 电控接口建议

如果你们现在已经确定电控侧会直接给二级 yaw 角度，那么文档里建议把接口约束写清楚：

1. 角度单位到底是 `rad` 还是 `deg`
2. 零点定义是什么
3. 正方向是顺时针还是逆时针
4. 这个角度描述的是“电机轴角度”还是“最终枪管相对一级 yaw 的关节角”

第一版最推荐的是：

- 电控直接给“二级 yaw 关节相对零位角”
- 视觉侧内部统一换成弧度
- 融合层只负责把这个角度乘到姿态链里

这样接口最干净，也最不容易和 IMU 四元数的坐标定义混在一起。

---

## 6. 建议复用哪些已有代码

### 6.1 IMU 入口

优先继续沿用 `standard` 里现成的：

- `io::CBoard`
- `cboard.imu_at(...)`

第一版不要把主链切去别的 IMU 线程，也不要为了双 yaw 先引入另一套视觉入口。

### 6.2 视觉主链

这些都尽量不要动：

- `solvePnP`
- 世界系目标位置计算
- `Tracker`
- `Aimer`
- `Shooter`

因为这些模块本来就在吃 `Solver` 输出的姿态结果。  
只要前面的姿态是对的，后面大部分逻辑都能直接复用。

### 6.3 电机数据

双 yaw 第一版真正必须新增的，只有：

1. 电控直接给的二级 yaw 角度 `theta_yaw2`

可选再加：

2. 二级 yaw 电机角速度 `omega_yaw2`

其中：

- `theta_yaw2` 是必须项，没有它就无法闭合姿态
- `omega_yaw2` 只是在有延迟补偿、预测前馈时更有用，不是第一步必须

---

## 7. 配置和链路上的最小收口建议

如果目标是“先最小改动跑起来”，建议第一阶段这样收口：

1. 二进制仍然用 `standard`
2. 视觉主链仍然用 `Camera + YOLO + Solver + Tracker + Aimer + Shooter`
3. 命令发送接口仍然走 `CBoard`
4. 先把反馈打开，让 `q` 变成真实 IMU 四元数
5. 把电控直接给的二级 yaw 角度补到姿态融合层

配置上，至少要满足：

- `can_send_only: false`
- `use_gimbal_feedback: true`

`aim_mode` 第一阶段可以先不强行大改。

如果你只是想先验证姿态融合本身，保留当前 `standard` 输出习惯也可以。  
等融合稳定之后，再决定是否把整条链切到更完整的绝对角模式。

---

## 8. 不建议的做法

第一版不建议做下面这些事：

- 不要先改 `Tracker / Aimer / Shooter` 内部逻辑
- 不要只在发送指令时额外补一个 yaw 偏置
- 不要只在开火判定里补“运动方向”
- 不要一开始就另起一条全新的 IMU/视觉线程

原因很简单：

- 根因在姿态入口，不在后级
- 后级补偿只能修现象，不能修世界系解算
- 新开链路会让改动面和联调成本一下变大

---

## 9. 最简单的验证方法

先做一个台架测试，不改算法就能判断方向对不对。

### 测试方法

1. 固定底盘和 IMU 所在主体
2. 只转枪管本体的二级 yaw
3. 同时记录：
   - IMU 解出来的 yaw
   - 电控给过来的二级 yaw 角度

### 判定

如果二级 yaw 转了，但 IMU yaw 基本不变：

- 说明 IMU 没看到最终枪管姿态
- 就应该按本文方案做融合

如果二级 yaw 一转，IMU yaw 也同步完整变化：

- 说明 IMU 已经看到了最终姿态
- 那就不要再把电机 yaw 重复乘进来

---

## 10. 一句话结论

按 `fuck.sh` 这条真实链路看，双 yaw 轴的最小改动方案就是：

**保留现有 IMU 和视觉主链，只在 `cboard.imu_at(...)` 到 `solver.set_R_gimbal2world(...)` 之间加一层“IMU 四元数 + 电控直接给的二级 yaw 角度”的姿态融合。**

后面的 `Solver / Tracker / Aimer / Shooter` 先都不动。
