# “依然不行”：首发故障与 DDS 对照

2026-10-31。此前发布异常处理修复不等于完整追球控制已实现；把
`start_chase:=true` 给成可用追球启动不准确。此次检查真实用户日志，
分别复测系统 Fast DDS、私有严格 Fast DDS、私有 Cyclone DDS。
没有放宽 watchdog，没有忽略转向请求，也没有自动清除故障。

## 用户运行的停机顺序

原始节点日志副本：`results/user-chase-stop-evidence.json`。

| ROS 日志时间（s） | 事件 |
|---:|---|
| 1793515435.990128 | C++ 验证启用后释放吊架 |
| 1793515436.909489 | policy 首先报 `turn_required_not_implemented` |
| 1793515436.992629 | 83 ms 后，WBC 报输入超时并禁用电机 |
| 1793515436.993447 | 仿真因禁用关节暂停 physics |
| 1793515437.241108 | 此后 Safety 才报 IMU age=256 ms |

这次 IMU 超时是停止刷新控制、暂停 physics 之后的后续故障。
模拟 IMU 有意不把冻结样本重新盖时间戳，不能通过继续发布“假新样本”消除它。
用户其他运行也存在独立的 DDS 启动等待，不能将所有 IMU 故障都解释成次生。

## 同一启动的复测

均采用正常 headless RGB-D/WBC 路径、`localhost_only:=0`、domain=221，
只在仿真中显式 `start_chase:=true`；保留所有试验。

- `chase-failure-repro-strict`：IMU/Safety 健康、WBC 2689 次，但 C++ 启动时
  发布错误已锁存，未放开吊架。证明“policy/WBC 还在运行”不能证明能启用。
- `chase-failure-repro-default`：原版 DDS 启动阶段 IMU age=333 ms，Safety
  锁存，未放开吊架。这是独立的启动通信/调度故障。

## 私有 Cyclone DDS 对照

仅将 ROS Jazzy 包下载并解压到 `generated/cyclone_trial/root`，没有 apt
安装、没有改 `/opt/ros`。使用 rmw_cyclonedds_cpp 2.2.4、Cyclone DDS 0.10.5，
以及包所需的私有 iceoryx 动态库；普通 ROS 库仍来自系统安装。
每个进程的 maps 确认加载了私有 `librmw_cyclonedds_cpp`、`libddsc`，并用
`rclpy.get_rmw_implementation_identifier()` 确认了运行实现。

固定吊架、电机禁用，串行运行三次、每次 32 秒：

| 记录 | IMU 发布最大间隙 | IMU 单次发布最长 | WBC 次数 | Safety/发布错误 |
|---|---:|---:|---:|---|
| cyclone-held-1 | 142.58 ms | 2.61 ms | 2775 | clear / 无错误 |
| cyclone-held-2 | 150.14 ms | 1.19 ms | 2785 | clear / 无错误 |
| cyclone-held-3 | 121.86 ms | 1.79 ms | 2790 | clear / 无错误 |

三次 policy 都保持运行，Safety 历史中没有锁存。仍有约 120–150 ms 启动
间隙，观察到的最大源龄约 118–200 ms，不是硬实时或长期可靠性保证。
这是固定吊架通信结果，不是行走成功率，也不是纯单参数库内锁实验。

然后执行一次显式放开吊架的 `cyclone-release-1`：

- 1793515884.123666：释放吊架。
- 1793515885.345678：1.222 秒后首先报 `turn_required_not_implemented`。
- 1793515885.423039：WBC 输入超时。
- 1793515885.428056：physics 因禁用关节暂停。
- 1793515885.686804：之后 IMU age=252 ms，Safety 锁存。

转向停止时，球检测仍有效，置信度约 0.925；球在机身坐标前方 1.302 m、
侧方 0.355 m，视觉正确请求 `vx=0.2 m/s, wz=0.400 rad/s`。
实验策略在 `abs(wz)>0.25` 时停止，因为它只改变原 ONNX 的关节参考幅度，
没有实现 yaw 控制。去掉这个检查不会产生转向能力。
因此 Cyclone 改善了这组启动，但完整追球仍被现有动作能力阻塞。

## 本次诊断改进

1. C++ 的全局传输故障状态使用明确 ID `cpp_control_transport_fault`；
   autostart 收到后立即报告实际故障并结束，不再误报等待 fresh commands 超时。
2. chase 停止后，保留首发 `chase_reason`，后续 simulator fault、estop 和
   命令过期不会覆盖它。原先 `fault` 已保留，但 `reason` 会被覆盖。
3. 测量工具默认仍固定吊架；需要显式 `--start-chase true` 才释放，并保存此参数。
4. README 首条命令改为固定吊架预览，明确说明转向和稳定站停尚未实现。

上述两条运行诊断更改均先复现失败测试，再修复。测试与构建日志为
`chase-first-reason-before-v2.log`、`chase-stop-regressions.log`、
`chase-stop-cpp-regressions.log`、`chase-failure-diagnostic-build.log`。

## 当前可验证的启动

在新终端执行固定吊架通信/相机预览：

```bash
cd /home/rcli/Github/Humanoid_ros2_simulation
source scripts/activate_rosenv.sh
source experimental/reference_warp/activate_cyclone_trial.sh
MUJOCO_GL=egl ros2 launch "$PWD/experimental/reference_warp/chase.launch.py" \
  start_chase:=false use_viewer:=false localhost_only:=0
```

若私有包丢失，可先执行 `bash experimental/reference_warp/prepare_cyclone_trial.sh`。
新开终端而不加载 `activate_cyclone_trial.sh` 即恢复系统默认 DDS。
严格 Fast DDS 的 XML 配置不用于 Cyclone。

要完成实际追球，仍需提供可执行转向的运动策略或带落脚点/接触规划的 WBC
步态生成；当前固定参考 ONNX 加小幅参考变形并不具备该能力。A* 只产生路径，
不能补上这层执行能力。这部分尚未完成，不能把本次诊断写成追球已修好。
