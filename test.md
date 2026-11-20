# 测试计划 / Test plan

要测什么、怎么测、怎样算通过、现在测到哪了。
What to test, how to test it, what counts as a pass, and where each item stands.

范围：本次从 `sim-experimental-cyclonedds` 移植来的功能（视觉锁定、GPU 前处理、VIO、控制链路诊断、safety 参数修复、IMU 离群处理），加上后续硬件选型需要的验证。已有功能的回归测试见 [README.md](README.md#features-and-how-to-test-them)。
Scope: the features ported from `sim-experimental-cyclonedds` (detector lock-on, GPU preprocessing, VIO, command-path diagnostics, safety override fix, IMU outlier handling), plus the checks that hardware choices still depend on. Regression tests for existing features are in [README.md](README.md#features-and-how-to-test-them).

> 通过标准里的数字，凡标注"建议"的都是提议的门槛，不是已测得的结果；其余数字是 2026-11-19 实测值。
> Numbers marked "proposed" in the pass criteria are suggested thresholds, not measured results; other numbers were measured on 2026-11-19.

## 测试环境 / Test setup

- Jetson Orin Nano Super，JetPack 7.2.1（L4T R39.2.1），Ubuntu 24.04，ROS 2 Jazzy，CycloneDDS `local` 配置。
  Jetson Orin Nano Super, JetPack 7.2.1 (L4T R39.2.1), Ubuntu 24.04, ROS 2 Jazzy, CycloneDDS `local` profile.
- 台架设备：RealSense D435（USB 3），ESP32-S3 + MPU6050（`/dev/ttyACM0`，policy_imu JSON 固件，200 Hz），vcan `can1`（没有电机）。
  Bench devices: RealSense D435 (USB 3), ESP32-S3 + MPU6050 (`/dev/ttyACM0`, policy_imu JSON firmware, 200 Hz), vcan `can1` (no motors).
- 比较 CPU 占用时先锁频：`sudo jetson_clocks --store <file>`（文件必须不存在，否则会卡在确认提示上），测完用 `sudo jetson_clocks --restore <file>` 恢复。
  Lock clocks before comparing CPU load: `sudo jetson_clocks --store <file>` (the file must not exist, or it waits on a prompt), then restore with `sudo jetson_clocks --restore <file>`.
- 每次测试用独立的 `ROS_DOMAIN_ID`，结束后确认没有残留进程：`pgrep -af '[r]ealsense2_camera_node|[b]all_detection_node|[v]io_localizer'`。
  Use a separate `ROS_DOMAIN_ID` per run and check for leftover processes afterwards: `pgrep -af '[r]ealsense2_camera_node|[b]all_detection_node|[v]io_localizer'`.

## 状态总览 / Status summary

| # | 项目 / Item | 状态 / Status |
|---|---|---|
| 1 | 单元测试 / Unit tests | 已通过 / passed (2026-11-19) |
| 2 | GPU 前处理等价性 / GPU preprocessing equivalence | 已通过 / passed |
| 3 | 检测器帧间锁定 / Detector lock-on | 合成图像流通过；**真球未测** / synthetic stream passed; **no real ball yet** |
| 4 | 带视觉的控制链路负载 / Control-path load with vision | 30 fps 和 60 fps 通过 / passed at 30 and 60 fps |
| 5 | safety 覆盖参数 / Safety launch overrides | Jazzy 通过；**Humble 未测** / passed on Jazzy; **Humble not tested** |
| 6 | IMU 加速度离群 skip 模式 / IMU outlier skip mode | 单元测试通过；**真 IMU 未测** / unit tests passed; **not on a real IMU** |
| 7 | 控制链路时延诊断 / Command-path timing diagnostics | 只编译过 / build only |
| 8 | VIO 静止 / VIO stationary | 已通过 / passed |
| 9 | VIO 运动 / VIO in motion | **未测** / **not tested** |
| 10 | VIO 与视觉、控制同时运行 / VIO combined load | **未测** / **not tested** |
| 11 | 机身 IMU 姿态（BNO085 / BMI088）/ Body IMU attitude | **未测** / **not tested** |
| 12 | 头部 IMU 与相机融合 / Head IMU fusion | **未开始**，等硬件选型 / **not started**, waits on hardware |
| 13 | 头部电机角度反馈 / Head actuator feedback | **未开始**，等硬件选型 / **not started**, waits on hardware |

---

## 1. 单元测试 / Unit tests

```bash
colcon build --packages-select humanoid_vision humanoid_localization motor_control_hybrid humanoid_safety \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ctest --test-dir build/humanoid_vision --output-on-failure            # yolo_face_cpu, target_tracker
python3 -m pytest humanoid_control/motor_control_hybrid/test/test_tracking_node.py
python3 -m pytest humanoid_control/humanoid_localization/test
```

- 通过 / Pass：全部通过。2026-11-19 结果：ctest 2/2，tracking node 10/10，localization 7/7。
  All pass. 2026-11-19: ctest 2/2, tracking node 10/10, localization 7/7.
- 已知问题 / Known issues：`test_motor_monitor.py::test_config_hot_reload...` 偶发失败（改动前就存在）；`motor_control_hybrid` 的 lint 检查（flake8、cpplint 等）整包失败，也是原有问题。
  `test_motor_monitor.py::test_config_hot_reload...` fails intermittently (pre-existing); the `motor_control_hybrid` lint checks (flake8, cpplint, ...) fail package-wide, also pre-existing.

## 2. GPU 前处理等价性 / GPU preprocessing equivalence

uint8 输入的 engine 必须和 FP32 输入的 engine 检测结果一致。
The uint8-input engine must give the same detections as the FP32-input engine.

```bash
# td-perception.onnx：sim-experimental-cyclonedds 的 simulation/humanoid_ball_chase/assets/
# td-perception.onnx: simulation/humanoid_ball_chase/assets/ on sim-experimental-cyclonedds
python3 scripts/make_u8_input_onnx.py td-perception.onnx td-perception-u8.onnx
trtexec --onnx=td-perception-u8.onnx --fp16 --builderOptimizationLevel=3 \
  --saveEngine=.deps/tensorrt/td-perception-u8-fp16.engine
```

- 用同一张有球的图分别跑两个 engine，比较类别、置信度和框。
  Run both engines on the same image with a ball; compare class, confidence and box.
- 通过 / Pass：类别和框完全一致，置信度差 < 0.001（建议）。实测：class 0，0.828937，框 (411,270,31,34) 完全一致；10.26 → 7.34 ms/帧。
  Same class and box, confidence within 0.001 (proposed). Measured: class 0, 0.828937, box (411,270,31,34) identical; 10.26 → 7.34 ms per frame.
- 注意 / Note：TensorRT 10.16 在优化级别 0 时会触发内部断言，要用级别 3。换 GPU 或 TensorRT 版本后要重建 engine。
  TensorRT 10.16 fails an internal assertion at optimization level 0; use level 3. Rebuild the engine after changing GPU or TensorRT version.

## 3. 检测器帧间锁定 / Detector target lock-on

```bash
E=$PWD/.deps/tensorrt/td-perception-u8-fp16.engine
ros2 launch humanoid_vision head_tracking.launch.py start_agent:=false start_tracker:=false \
  model_path:=$E target_class_id:=0 color_profile:=640,480,60 show_image:=true
ros2 topic echo /vision/target_track
```

已做 / Done：用一张截图循环喂 60 Hz 图像，覆盖静止、平移、遮挡 3 帧、遮挡 0.5 s 四种情况，行为都符合设计。
Fed a 60 Hz loop of one screenshot through four phases (still, panning, 3-frame occlusion, 0.5 s occlusion); all matched the design.

真球待测 / Real-ball tests to do:

| 场景 / Scenario | 做法 / How | 通过 / Pass |
|---|---|---|
| 新目标确认 / New target | 球放进画面 / Put a ball in view | 第 2 帧开始输出，id 不变 / output from the 2nd frame, id stays the same |
| 干扰球 / Distractor | 锁定后再放第二个球 / Add a second ball after lock-on | id 不变，中心不跳到第二个球 / id unchanged, center does not jump |
| 短遮挡 / Short occlusion | 手挡 < 0.1 s / Cover < 0.1 s | `score=0` 预测几帧后恢复，id 不变 / a few `score=0` predictions, then same id |
| 长遮挡 / Long occlusion | 挡 > 0.1 s / Cover > 0.1 s | 停止输出，再出现时 id+1 / output stops, id increments on reappearance |
| 误检 / False positives | 场地上放机器人、球门、标线 / Robots, goal and lines in view, no ball | 不输出 / no output |
| 抖动 / Jitter | 球静止 30 s，记录 `/vision/target_center` / Ball still 30 s, record the center | 标准差比 `track_target:=false` 时小（建议 ≤ 0.6 倍）/ std dev below `track_target:=false` (proposed ≤ 0.6×) |
| 远近 / Range | 球放在 1、2、3、4 m / Ball at 1, 2, 3, 4 m | 记录检出率和框宽，验证测距公式 `Z = fx × 0.20 / 框宽` / record detection rate and box width; check `Z = fx × 0.20 / width` |

## 4. 带视觉的控制链路负载 / Control-path load with vision

检测器、相机和控制栈同时运行时，控制话题的频率和抖动不能变差。
With the detector, camera and control stack running together, control topic rates and jitter must not degrade.

```bash
sudo modprobe vcan; sudo ip link add can1 type vcan; sudo ip link set can1 up
# vcan 专用的电机配置副本；不要修改真实的 motors.yaml
# A vcan-only copy of the motor registry; never edit the real motors.yaml
sed "s/hardware_verified: false/hardware_verified: true/" \
  humanoid_control/motor_control_hybrid/config/motors.yaml > /tmp/motors_vcan_only.yaml
ros2 launch motor_control_hybrid hybrid_control.launch.py enable_fake_motor:=false \
  enable_safety:=true enable_cpp_control:=true motor_config_file:=/tmp/motors_vcan_only.yaml &
ros2 launch motor_control_hybrid tracking_policy.launch.py start_imu_reader:=true port:=/dev/ttyACM0 \
  baud:=460800 input_format:=json orientation_source:=complementary \
  use_fake_joint_states:=true joint_feedback_mode:=joint_states run_cpp_control:=false &
ros2 run humanoid_safety fall_predictor_node &
ros2 launch humanoid_vision head_tracking.launch.py start_agent:=false \
  model_path:=$E target_class_id:=0 color_profile:=640,480,60 &
# 各话题：ros2 topic hz --window 500 <topic>；CPU：pidstat；GPU 和功耗：tegrastats
# Per topic: ros2 topic hz --window 500 <topic>; CPU: pidstat; GPU and power: tegrastats
```

| 话题 / Topic | 通过 / Pass | 60 fps 实测 / Measured at 60 fps |
|---|---|---|
| `/imu/data_raw` | 200 Hz，最大间隔 ≤ 10 ms（建议）/ max gap ≤ 10 ms (proposed) | 199.98 Hz，sd 0.97 ms，max 9 ms |
| `/safety/status` | 100 Hz，最大间隔 ≤ 15 ms（建议）/ max gap ≤ 15 ms (proposed) | 99.98 Hz，sd 0.97 ms，max 13 ms |
| `/motor_commands` | 50 Hz | 50.00 Hz，sd 0.40 ms |
| 检测 / Detection | ≥ 相机帧率的 98% / ≥ 98% of camera rate | 59.5 FPS，7.9 ms/帧 / per frame |

- 记录系统 CPU、GPU 和整机功耗，60 fps 时分别为 50%、29%、10.3 W。
  Record system CPU, GPU and board power: 50%, 29% and 10.3 W at 60 fps.
- 每次都要确认开始前没有残留节点；`realsense2_camera_node` 退出时可能卡住，需要 `kill -9`。
  Confirm no leftover nodes before each run; `realsense2_camera_node` can hang on exit and need `kill -9`.

## 5. safety 覆盖参数 / Safety launch overrides

```bash
for args in "" "fall_detection_enabled:=false trip_on_imu_timeout:=false"; do
  ros2 launch humanoid_safety safety.launch.py $args &
  sleep 6
  ros2 param get /safety_node fall_detection_enabled
  ros2 param get /safety_node trip_on_imu_timeout
  kill %1
done
```

- 通过 / Pass：不传参数时用 YAML 的值（true），传参数时改为 false。Jazzy 已通过。
  No override keeps the YAML value (true); an override sets false. Passed on Jazzy.
- 待测 / To do：在 Humble（JetPack 6）上重复。这个修复针对的是 Humble 上覆盖参数被忽略的问题。
  Repeat on Humble (JetPack 6); the fix targets overrides being ignored there.

## 6. IMU 加速度离群 skip 模式 / IMU acceleration outlier skip mode

`imu_accel_outlier` 默认是 `latch`，和原来的行为一样。`skip` 只在显式设置时生效。这个参数目前没有在 launch 里暴露，要直接给节点传参。
`imu_accel_outlier` defaults to `latch`, the previous behavior; `skip` applies only when set. It is not exposed in the launch file yet: pass it to the node directly.

```bash
ros2 run motor_control_hybrid tracking_policy_node --ros-args -p imu_accel_outlier:=skip
ros2 topic echo --field data /policy/status    # imu_accel_outliers
```

- 待测 / To do：在真 IMU 上轻敲 IMU 板或让它跌落到软垫上，观察 `imu_accel_outliers` 计数。孤立的尖峰不应锁死；持续超过 `input_timeout_s` 应锁死，报 `Invalid IMU acceleration persisted`。
  On a real IMU, tap the board or drop it onto a pad and watch `imu_accel_outliers`. An isolated spike must not latch; outliers lasting longer than `input_timeout_s` must latch with `Invalid IMU acceleration persisted`.
- 同时确认 `latch` 模式下第一个离群样本就锁死。
  Also confirm that `latch` mode latches on the first outlier.

## 7. 控制链路时延诊断 / Command-path timing diagnostics

```bash
HUMANOID_CPP_DIAG=1 ros2 launch motor_control_hybrid hybrid_control.launch.py ...   # stderr: CPPDIAG {...}
```

- 通过 / Pass：每秒打印一行 `CPPDIAG`，包含 `rx_age_ms`、`tx_age_ms`、`loop_gap_ms`；不设环境变量时不输出，也不影响控制。
  One `CPPDIAG` line per second with `rx_age_ms`, `tx_age_ms`, `loop_gap_ms`; nothing is printed without the variable, and control is unaffected.
- 用途 / Use：在第 4 项负载测试中打开，定位命令过期发生在哪一段。
  Turn it on during the load test (item 4) to locate where commands expire.

## 8. VIO 静止 / VIO stationary

```bash
# 终端 1：相机红外双目（投射器关闭）、安装 TF、vio_localizer
# Terminal 1: D435 IR pair (projector off), mount TF, vio_localizer
ros2 launch humanoid_localization vio_d435.launch.py \
  camera_x:=0 camera_y:=0 camera_z:=0 camera_roll:=0 camera_pitch:=0 camera_yaw:=0
# 终端 2：容器里的 cuVSLAM / Terminal 2: cuVSLAM in the container
scripts/run_cuvslam.sh
ros2 param get /camera/camera depth_module.emitter_enabled          # 0
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic echo --field data /localization/status
```

- 通过 / Pass：两路红外各 30 Hz，投射器参数为 0，VO 30 Hz，`/localization/odom` 50 Hz，静止 60 s 位置漂移 < 1 cm（建议）。
  IR 30 Hz each, projector 0, VO 30 Hz, `/localization/odom` 50 Hz, position drift < 1 cm over 60 s stationary (proposed).
- 实测 / Measured：全部达到；3835 步被接受、0 步被拒；位置 (0.000, 0.000, 0.000) m；丢了 4 帧（相邻帧间隔 66.7 ms）。
  All met; 3835 steps accepted, 0 rejected; position (0.000, 0.000, 0.000) m; 4 dropped frames (66.7 ms gaps).

## 9. VIO 运动 / VIO in motion

先手持相机测，再装到机器人上测。安装参数要先在机器人上量好。
Test hand-held first, then on the robot. Measure the camera mount on the robot first.

| 场景 / Scenario | 做法 / How | 通过 / Pass |
|---|---|---|
| 闭环 / Closed loop | 沿已知路线（如 5 m × 5 m）走一圈回到起点 / Walk a known loop (e.g. 5 m × 5 m) back to the start | 终点误差 < 路程的 3%（建议；仿真为 1.6–2.6%）/ end error < 3% of distance (proposed; simulation gave 1.6–2.6%) |
| 直线 / Straight line | 用卷尺量 3 m 直线 / Measure a 3 m line with a tape | 误差 < 10 cm（建议）/ error < 10 cm (proposed) |
| 快速转头 / Fast head turns | 头部来回转 ±60° / Pan ±60° back and forth | 不出现 `rejected` 或 `restarted`；如果出现，记录角速度 / no `rejected` or `restarted`; if they occur, note the turn rate |
| 原地转身 / Turn in place | 转 360° / Rotate 360° | 回到原朝向时 yaw 误差 < 5°（建议）/ yaw error < 5° on return (proposed) |
| 暗光 / Low light | 关掉部分室内灯 / Turn off some room lights | 记录跟踪还能维持到多暗 / record how dark tracking still holds |
| 少纹理 / Low texture | 对着白墙或纯色地毯 / Face a plain wall or carpet | 记录跟丢和恢复的情况 / record loss and recovery |
| 遮挡恢复 / Occlusion recovery | 挡住镜头 1 s / Cover the lens for 1 s | 出现 `restarted`，位置不跳，之后继续累加 / `restarted` occurs, position does not jump, then accumulates again |

- 每个场景都记录 `/localization/status` 和 cuVSLAM 日志里的丢帧警告。
  For each scenario, record `/localization/status` and the dropped-frame warnings in the cuVSLAM log.
- 头部安装 TF 是静态的，转头测试时位置会有误差，这属于已知限制，要等第 13 项完成。
  The head mount TF is static, so head-turn runs carry a known position error until item 13 is done.

## 10. VIO 与视觉、控制同时运行 / VIO combined load

第 4 项全部节点加上第 8 项的 VIO。相机只能由一个进程打开：用一个 `realsense2_camera_node` 同时开彩色流和红外流，`vio_d435.launch.py` 传 `start_camera:=false`。
All of item 4 plus the VIO from item 8. Only one process may own the camera: run one `realsense2_camera_node` with both color and IR, and pass `start_camera:=false` to `vio_d435.launch.py`.

- 通过 / Pass：第 4 项的标准全部满足；VO 保持 30 Hz；检测每帧耗时增加 < 30%（建议）。
  All item 4 criteria hold; VO stays at 30 Hz; detection time per frame grows < 30% (proposed).
- 如果 GPU 不够，先把检测降到 30 fps，不要降低控制频率。
  If the GPU is short, lower detection to 30 fps; never lower control rates.

## 11. 机身 IMU 姿态 / Body IMU attitude

决定 VIO 的 `attitude_source:=imu` 能否使用，以及机身 IMU 用 BNO085 还是 BMI088。
Decides whether VIO `attitude_source:=imu` is usable, and BNO085 versus BMI088 for the body.

- 把两块 IMU 装在同一块板上，固定在机身上；一起录静止、手动倾斜、走路和落地冲击的数据。
  Mount both IMUs on one plate on the body; record stationary, hand-tilted, walking and foot-impact data together.
- 比较 / Compare：静止 10 min 的 roll/pitch 漂移和 yaw 漂移；倾斜到已知角度（用量角器或倾角仪）时的误差；冲击后姿态偏离的大小和恢复时间。
  Roll/pitch and yaw drift over 10 min stationary; error at known tilt angles (protractor or inclinometer); size and recovery time of attitude offsets after impacts.
- BMI088 需要自己做融合（RK4 + Mahony，见 `origin/velocity-handoff:utils/imu_read/imu_read.py`），门限要用走路数据重新调。
  BMI088 needs our own fusion (RK4 + Mahony, see `origin/velocity-handoff:utils/imu_read/imu_read.py`); retune its gates on walking data.
- 改用 `attitude_source:=imu` 前，先确认 IMU 轴向和 `base_link` 对齐，或者给 `vio_localizer` 加上安装旋转参数；并确认 micro-ROS 时间戳和 Jetson 时钟的偏差远小于 50 ms。
  Before using `attitude_source:=imu`, align the IMU axes with `base_link` or add a mounting-rotation parameter to `vio_localizer`, and check that micro-ROS stamps are well within 50 ms of the Jetson clock.

## 12. 头部 IMU 与相机融合 / Head IMU fusion

等硬件选型（D435i，或头部板上接 BMI088）。这一项只在第 9 项发现纯视觉跟丢严重时才需要。
Waits on the hardware choice (D435i, or a BMI088 on the head board). Needed only if item 9 shows frequent VO loss.

- 外参 / Extrinsic：D435i 用出厂外参；自装 IMU 用 Kalibr 标定。
  D435i uses its factory extrinsic; a separate IMU needs Kalibr calibration.
- 时钟 / Clock：测 IMU 和图像时间戳的偏差和抖动，抖动必须 < 10 ms（cuVSLAM 的门限）。
  Measure the offset and jitter between IMU and image stamps; jitter must stay under 10 ms (cuVSLAM's limit).
- 用 `scripts/run_cuvslam.sh imu_fusion:=true` 重复第 9 项，和纯视觉对比。
  Repeat item 9 with `scripts/run_cuvslam.sh imu_fusion:=true` and compare with pure VO.

## 13. 头部电机角度反馈 / Head actuator feedback

等硬件选型（如飞特 STS3215 两轴）。目标是让 `base_link → camera_link` 随头部实际角度变化。
Waits on the hardware choice (for example a two-axis Feetech STS3215). The goal is a `base_link → camera_link` transform that follows the measured head angle.

- 角度精度 / Angle accuracy：命令几个已知角度，用外部参考（量角器或相机看标定板）对比读回角度。
  Command known angles; compare the read-back angle with an external reference (protractor, or the camera viewing a calibration board).
- 齿隙 / Backlash：从两个方向转到同一角度，比较两次读数的差。
  Approach the same angle from both directions; compare the readings.
- 时间戳 / Timing：角度消息的时间戳和 Jetson 时钟对齐，读回频率 ≥ 50 Hz（建议）。
  Angle message stamps aligned with the Jetson clock; read-back rate ≥ 50 Hz (proposed).
- 接上 `robot_state_publisher` 后，重复第 9 项的快速转头场景，位置误差应明显小于用静态 TF 时。
  With `robot_state_publisher` wired in, repeat item 9's head-turn scenario; position error should be clearly lower than with the static TF.
