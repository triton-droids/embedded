# 交接：VIO 定位、Isaac ROS 检测与仿真负载问题

仓库 `triton-droids/embedded`，分支 `sim-experimental-cyclonedds`。本文写于
2026-11-12，之前的工作截至提交 `2c6ea989`，已推送。

## 1. 已经有的

### 1.1 定位（不用仿真真值）
- 包 `humanoid_control/humanoid_localization`（README 有详细说明）：
  - `vio_localizer`：姿态取机身 IMU（模拟 BNO085 game rotation vector，无磁力计），
    位置由 cuVSLAM 双目 VO 的位移经头部 TF（base→stereo_rig）换算后累加；
    速度门限 2 m/s；起点是已知开球位姿（第一帧 VO 时初始化）。
  - `localization_eval`（与真值比对，仅评估用）、`route_driver`（仿真里挪球带机器人
    走 16 段左转路线，定位初始化前把球藏在身后）。
  - cuVSLAM 跑在 Isaac ROS 3.2 容器里：`scripts/run_cuvslam.sh build|<run>`。
- 仿真侧（`simulation/humanoid_sim`）：
  - `enable_stereo`：D435 两个红外相机（被动，投射器关）的模拟，50 mm 基线，
    640×480 mono8 15 Hz，带 camera_info 和 TF。
  - `stereo_split`（默认）：双目在独立进程 `sim_stereo_publisher` 渲染，状态经共享
    内存 `state_shm`（按 ROS_DOMAIN_ID 分文件）传递。
  - `share_state`：不开双目也写共享内存，给录制用。
  - `imu_yaw_error_deg/period_s`：给模拟 IMU 加正弦航向误差。
- 精度（Jetson，困难场景，约 14 m 路线）：最终误差为路程的 1.6–2.6 %；
  误差主要来自场地西南角纹理少时转身的 VO 跳变。数据在 `results/`。

### 1.2 Isaac ROS YOLO 检测
- `simulation/humanoid_ball_chase/launch/isaac_yolo.launch.py`：编码器 → TensorRT →
  YOLOv8 解码器（NITROS，一个容器）。现有 td-perception / nubot YOLO11n 引擎不用改
  （输出布局同 YOLOv8）。`with_cuvslam:=true` 把 cuVSLAM 也装进同一容器。
- 启动：`scripts/run_isaac_yolo.sh`（只检测）或 `scripts/run_isaac_perception.sh`
  （检测 + cuVSLAM 一个容器）；`ISAAC_CPUSET=4,5` 可绑核。
- 下游：`ball_chase_vision detector:=isaac`（launch：`vision_detector:=isaac
  vision_target_class:=N`）。按图像时间戳等对应检测（≤0.3 s），按类别选；同类多个
  实例时锁定离上一个目标最近的（相机系 3D 点），1 s 丢失后重选离相机最近的。
- td-perception 类别：0 球、1 球门柱、2 机器人、3 L/4 T/5 X 线交点、6 点球点。
- 与原 Python/TensorRT 检测器对比：同一引擎，球中心中位差 0.64 px，延迟相近。

### 1.3 评估与诊断工具
- `evaluation/vio_isaac_trials/`：试验/录制/渲染/分析脚本、结果 CSV、README。
- 计时诊断（默认全关）：
  - `HUMANOID_SIM_DIAG=1`（mujoco_sim，`humanoid_sim/diag.py`）
  - `HUMANOID_CPP_DIAG=1`（cpp_control_node）
  - `walk_slow_log_ms:=40`（walking_controller）
  - 说明和副作用见 `evaluation/vio_isaac_trials/README.md`。

### 1.4 环境
- 新 Jetson：`td@100.71.186.19`（Tailscale），Orin Nano 8 GB，JetPack 6.2.1，Humble。
  - 工作目录 `~/Documents/Github/embedded-vio`（分支 `vio-test`，rosenv/.deps 链接到
    `../embedded-sim-experimental`），镜像 `humanoid_cuvslam:3.2`（含 cuVSLAM + YOLO）。
  - 试验数据和脚本副本：`~/vio_runs/`。
  - 无 sudo；用户在 docker 组。
- 旧 Jetson `192.168.1.44` 目前连不上。
- 机器人头部相机是 D435（无 IMU），VO 用被动红外双目，投射器必须关。

## 2. 当前未解决的问题

### 2.1 仿真行走的时间余量太小（最高优先级）
- 原检测器、无额外负载：每次都走完路线（w5、w9、s2）。
- 加 Isaac 容器（各种组合，绑核也一样）或者原检测器 + 诊断：都会在转向时摔倒。
  检测本身正确；摔倒前的指令跳变是相机随身体倾倒而晃动的结果。
- 指令链路测量（d8，原检测器，走路中，每秒最大值的中位数）：
  控制器→cpp_control 收到 3 ms，cpp_control 转发时 23 ms，仿真收到时 29 ms；
  指令有效期 80 ms（walking_controller `output_lifetime_s`）。
  cpp_control 内部约 20 ms，原因不明（200 Hz 定时器、转发时取所有关节里最旧的
  source 时间戳、`cmd_mutex_` 等都可能）。
  摔倒那一秒最后一跳（cpp_control → 仿真）到 78 ms，而仿真进程 GIL/锁/物理步正常。

### 2.2 `sim_autostart` 经常超时
- 报错 `Release denied: all motors need fresh motion commands; enabled=[]`。
- d1、d3–d7 都出现（d7 没有日志刷屏也出现）。启动阶段仿真进程 GIL 占用最长
  580 ms、单个物理步最长 161 ms。

### 2.3 诊断工具会干扰被测系统
- `walk_slow_log_ms:=10` 会刷屏拖慢控制器，导致 autostart 超时；用 40。
- `HUMANOID_SIM_DIAG` 的 GIL 探测线程（每 2 ms 唤醒）本身增加 GIL 竞争。
- 教训：`ros2 topic echo` 在话题出现前启动要显式写类型；给已有类加属性前先 grep
  同名（曾因 `_last_tick_mono` 重名让控制器崩溃）。

## 3. 下一步建议（按优先级）

1. **查清 cpp_control_node 内部约 20 ms。**
   读 `humanoid_control/motor_control_hybrid/src/cpp_control_node.cpp` 的
   `desired_motor_callback`、`control_loop`、`publish_output`（转发时取最旧 source）。
   目标：转发时的指令年龄降到 ≤ 收到时 + 1 个周期（约 8 ms）。
   验收：`HUMANOID_CPP_DIAG=1` 下 `tx_age_ms - rx_age_ms` 中位数 < 8 ms。
2. **查最后一跳偶发的长延迟和启动卡顿。** 仿真的指令接收在 `mujoco_sim` 的专用
   executor（`dedicated_command_executor`）；考虑把指令接收也移出 GIL 所在进程
   （类似 IMU 的 `imu_split` 做法），或用更轻量的方式测（避免 GIL 探测线程）。
   验收：开 Isaac 容器时 10 段路线能走完，`cmd_age_ms` 最大值 < 60 ms。
3. **修 `sim_autostart` 超时**：先确认 ENABLE 是否被执行、为什么 `enabled=[]`。
4. **定位质量**（不依赖上面几项）：
   - 用 VO 的转角修正 IMU 航向（互补滤波/EKF），并丢弃与 IMU 转角不一致的 VO 帧；
     速度门限收紧到约 0.6 m/s。
   - 用 td-perception 的地标类（线交点、球门柱、点球点）+ 已知场地图做全局校正。
   - 可选：cuVSLAM 建图 + 地图内定位（`save_map`/`load_map`）。
5. **上真机前**：
   - base→stereo_rig 由头部舵机角度正运动学计算，并和图像时间戳对齐（仿真里现在
     直接取真值位姿）。
   - RealSense：`emitter_enabled:=0`，开 infra1/infra2，接到 cuVSLAM。
   - 确认真 BNO085 的话题、坐标系、协方差与 `/imu/data_raw` 一致。
6. **整理**：Isaac 检测器用于非球目标时，日志措辞和 `ball_radius`（深度+0.1 m）
   仍按球处理；清理 Jetson 上的旧数据。

## 4. 如何复现

在 Jetson 上（`~/vio_runs/vio_chase.sh`，选项见 evaluation README）：

```bash
VIO=0 DETECTOR=builtin ./vio_chase.sh NAME 120 10   # 原检测器，对照
VIO=0 ./vio_chase.sh NAME 120 10                    # Isaac 检测
./vio_chase.sh NAME 170 10                          # VIO + Isaac（一个容器）
python3 analyze_diag.py NAME                        # 在拷回的结果目录上分析
```

`sim.log` 里在 `signal_handler(SIGINT)` 之后出现的 `Physics paused` 是脚本收尾，不是故障。
