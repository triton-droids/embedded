# 私有 Fast DDS 严格实时构建对照

日期：2026-10-31。仅在本机仿真执行，未连接真实电机。

后续已补齐发布失败处理及故障注入回归，见
[PUBLISH_FAILURE_HANDLING.md](PUBLISH_FAILURE_HANDLING.md)。以下保留修复前的六次数据。

**结论：5 ms 的两次 IMU 流均持续到测试结束，但整个控制链只完成其中
一次；另一次 cpp_control_node 因未处理的 DDS 发布错误退出。不能默认启用。**

## 六次实测结果

所有时长为毫秒。原生耗时指真实转发探针覆盖的 RMW publish 调用。
“IMU 发布间隙”来自发布端诊断；如果发布已停止，该诊断也可能停止，
不能把最后一次诊断中的小间隙当作持续健康。

| 条件/轮次 | IMU 发布最大间隙 | IMU 原生发布最长 | 所有 topic 原生发布最长 | 最终结果 |
|---|---:|---:|---:|---|
| 原版 1 | 298.03 | 4.31 | 291.44 | WBC deadline fault，Safety 未锁存 |
| 原版 2 | 44.61 | 3.69 | 7.26 | 完成，WBC 2613 次 |
| strict 1 | 18.85* | 100.15 | 100.22 | IMU 在 t=5.47 s 后停止，Safety 锁存；视觉节点退出 |
| strict 2 | 51.59 | 0.09 | 104.24 | cpp_control_node 异常退出，WBC 等待输入 |
| strict5ms 1 | 24.26 | 5.62 | 9.48 | 完成，WBC 2602 次 |
| strict5ms 2 | 27.73 | 4.84 | 100.98 | cpp_control_node 异常退出，虽然 WBC 仍有 2618 次输出 |

\* strict 1 的发布状态是停止前的旧状态；观察器稳定阶段（12–32 s）收到
0 条 IMU，绝不能按 18.85 ms 宣称改善。

关键事件：

- 原版 1：`/head/tilt_target_rad` 等待 291.44 ms，`/joint_states` 等待
  277.91 ms；IMU 原生发布自身只有 4.31 ms，但与 joint_states 共进程，
  发布端 IMU 间隙达到 298.03 ms。WBC 另报总计算时间 10.405 ms 超期，
  不能直接认定其故障由这次 DDS 等待造成。
- strict 1：`/imu/data_raw` 在约 100.15 ms 返回 RMW 错误，IMU 随后
  停止；`/ball_chase/cmd_vel` 两次约 100 ms 错误，视觉节点异常退出。
  Safety 正常锁存 IMU 断流，没有放宽或重置它。
- strict 2 与 strict5ms 2：`/motor_commands` 在启动阶段分别约
  104.24/100.98 ms 返回错误；C++ 节点抛未捕获的 `rclcpp::exceptions::RCLError`
  并退出（-6）。该 writer 未被 5 ms XML 覆盖，仍是 100 ms。
  **WBC 仍运行并不代表 cpp_control_node 或输出链健康。**
- strict5ms 1 无记录到的原生发布错误，关键节点运行到观察结束。
  strict5ms 2 有一次 `/joint_states` 原生调用 21.18 ms 且返回成功；因此
  5 ms 是 DDS 内部等待参数，不是整个调用的绝对时限。

按“无 Safety 锁存、无控制故障、关键节点不提前退出、IMU 持续”的本次
观察标准，原版 1/2、strict 0/2、strict5ms 1/2。样本非常少且都处于吊架，
不能将其解释成可靠性概率或行走成功率。

12–32 s 稳定阶段，strict5ms 两次观察器接收最大间隙为 20.20/19.48 ms；
原版为 18.81/44.66 ms。strict5ms 的启动观察器仍有 342.63/331.11 ms
接收间隙，而发布端最大间隙只有 24.26/27.73 ms。这说明仅凭观察器延迟
不能定位 IMU 发布线程；接收端启动、发现和调度仍需分别看待。

每次记录到 13 个 DDS 进程的库映射：原版均为系统库，B/C 均为私有库，
未发现这三种被检查库在同一测试中混用。性能测试期间没有编译进程。
本次 CPU 平均值不用于比较优劣：部分控制节点失败使负载不同。

原始数据位于 `results/dds-tune-*.{imu,cpu}.json`、同名 `.log` 及
`*-trace/`；汇总为 [dds-tune-summary.json](results/dds-tune-summary.json)，
顺序/完成记录为 [dds-tune-runner.log](results/dds-tune-runner.log)。全部六次
及退出期间图形上下文错误均保留；退出后的错误未算成运行期间故障。

下一步应先补齐各发布路径的明确失败处理：传感器超时保留计数、时间戳并
等下一次新样本；控制命令发布失败必须使既有控制链进入可观测的安全故障，
不可静默忽略或自动重发过期命令。再测试发现阶段与稳定阶段、关键 writer
隔离和启动顺序。当前实验没有修改这些运行代码或正式默认配置。

## 实验设置

Fast DDS 2.14.5，rmw_fastrtps 8.4.3。私有构建使用 Release、
`STRICT_REALTIME=ON`，Security、SQLite3、Statistics 开启；使用从 Ubuntu
Noble libasio-dev 1:1.28.1-0.2 包解出的头文件。构建及安装均位于
`generated/dds_strict/`，没有覆盖 `/opt/ros/jazzy`。
匹配的 `rmw_fastrtps_shared_cpp` 和 `rmw_fastrtps_cpp` 使用相同版本源码重建。
CMake cache、编译 include 路径及实际进程 `/proc/PID/maps` 用于交叉检查。

| 条件 | DDS/RMW | IMU 与 joint_states max_blocking_time |
|---|---|---|
| baseline | 系统原版，安装头文件 strict=0 | 100 ms |
| strict | 私有严格实时构建，strict=1 | 100 ms |
| strict5ms | 私有严格实时构建，strict=1 | 5 ms |

`publish_probe/writer_qos.cpp` 在独立测试 domain 创建同名 writer，读取真实
DataWriter QoS，而非只读环境变量。两套构建分别编译 probe；
`qos-baseline.txt`、`qos-strict.txt`、`qos-strict5ms.txt` 保存回读与动态库路径。
全部回读均确认异步发布（1）、PREALLOCATED_WITH_REALLOC（1）、
KeepLast（0）、depth=10、Data-sharing OFF（3）。5 ms XML 仅指定
`/imu/data_raw` 与 `/joint_states`；probe 的 `/safety/estop` 等待值仍为 100 ms。
真实 Safety 急停 QoS 和代码未更改。

正常 `chase.launch.py` GUI 路径，`start_chase:=false`，吊架固定，电机禁用，
`localhost_only:=0`，domain=232；沿用 IMU > WBC > 其他的 nice/CPU 配置。
保留原 250 ms Safety IMU 超时及其他输入、命令期限。
每次独立启动观察 32 秒，六次串行顺序：A B C C B A。
每次都记录真实 rcl/rmw 发布调用、线程 CPU 时间、返回值、IMU 源龄/间隙、
Safety、policy、WBC 状态；长调用必录，常规调用抽样，不从抽样数据推算 p99。
启动/退出期间错误保留，不把程序正常退出当作控制成功。

## 复现

从未加载私有 DDS 的新 shell 执行：

```bash
cd /home/rcli/Github/Humanoid_ros2_simulation
python3 experimental/reference_warp/run_dds_tuning.py --label dds-new --rounds 2
python3 experimental/reference_warp/analyze_dds_tuning.py --label dds-new
```

`run_dds_tuning.py` 自动给 B/C 子进程加载私有环境。正式默认启动未改为私有版。
构建脚本为 `build_strict_fastdds.sh`、`build_strict_rmw.sh`；前者要求
`generated/dds_strict/` 内已准备源码及解出的 Asio 头文件。
手动测试私有环境时，先加载正常 `scripts/activate_rosenv.sh`，然后 source
`experimental/reference_warp/activate_strict_dds.sh`；新 shell 恢复原版。

## 解释边界

严格模式源码在 `DataWriterImpl::perform_create_new_change` 对 writer 锁使用
带期限获取，原版分支直接等待锁。但此前测到的 332 ms 只定位到原生 RMW
调用，未证明具体阻塞在此锁。严格模式并非整个发布调用的硬实时上界。

严格构建还默认把进程内直接投递从 FULL 改成 OFF，且这里是私有 Release
重建与系统发行版比较；因此 A/B 差异不能全部归因于单一锁分支。B/C 使用
同一套二进制，仅针对两个 writer 改等待上限。短期吊架测试也不代表自由
行走或真实机器人安全；若有超时异常退出，必须计为失败，不能静默忽略。
