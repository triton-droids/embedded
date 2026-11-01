# DDS 发布失败的处理与回归

2026-10-31。针对 [DDS_TUNING_RESULTS.md](DDS_TUNING_RESULTS.md) 实测的
未捕获发布异常，完成运行代码修复。严格 DDS 与 5 ms 配置仍仅供实验，
没有改成正式默认，也没有放宽 Safety、命令有效期或输入超时。

## 修复行为

- 模拟 IMU、关节反馈：捕获 `RCLError`，失败帧不算发布成功、不置 readiness、
  不缓存成已发布反馈；下一周期重新取物理采样，保留采样源时间。
  `/imu/status.publish_failures`、`/can_timing.publish_failures` 保存计数及最后错误。
  采样回调不直接打印失败日志，避免日志锁/ROS 日志通信再次阻塞采样线程。
  模拟器其他状态/遥测发布采用同样的失败计数机制。
- `cpp_control_node`：所有 `/motor_commands` 写入统一捕获 ROS 发布错误。
  第一次失败即锁存，清除缓存的运动/ENABLE、终止管理操作，立即尝试一个新
  DISABLE；若也失败，后续控制周期继续尝试新 DISABLE，不递归重试、不重发旧
  运动命令。新输入和 Safety reset 都不能清除该本地故障，需重启节点。
  `/control_core/abstract_command_status` 报 FAILED、错误计数以及最近禁用消息
  是否成功交给 DDS；这不等同于实际电机已禁用的反馈证明。
- WBC：输出发布错误锁存故障，停止生成运动输出并重试 DISABLE。
  policy 现有 worker 异常锁存继续保留；补齐诊断发布异常处理。
- chase：命令发布失败后锁存零请求，后续识别到新目标也不自动恢复追逐。
- 头部 yaw/pitch C++ tracker：发布失败后锁存停止输出，让现有 MCU/模拟
  舵机 watchdog 释放；节点保持存活，重新识别目标不能清除故障。
- Safety：急停写入失败不更新“已发送状态”，每次评估继续发送待交付状态；
  保持 Reliable/TransientLocal。诊断发布失败不能终止 watchdog 执行器。

这些更改处理的是“异常退出/线程死亡/错误恢复旧命令”，不保证 DDS 从此
不阻塞，也不保证网络故障时禁用命令一定送达。原有下游有效期、watchdog
和急停仍是独立约束。没有真实硬件/CAN 验证。

## 确定性回归

修复前，在真实回调的 publish 位置注入 `RCLError`，复现了 IMU 异常向外
传播、急停重试中断、chase 异常向外传播，以及 C++ 旧命令未清除/控制回调
抛异常。原始失败日志为 `results/dds-fix-*-before*.log`。
最初的 domain 233/234 不合法启动也保留，修正为 220/221 后才作为有效回归。

修复后：

- C++ watchdog/control 回归：13 项通过，包含“运动发布与第一次禁用连续
  失败，随后恢复只允许 DISABLE”的测试。
- Python 仿真、视觉、Safety、策略时间合同、WBC：266 项通过，3 项跳过。
  该组未运行可选 self_collision 目录。第一次更宽的测试发现 `coal` 缺失，
  相关碰撞测试失败/报错；没有为本任务安装或修改碰撞依赖。
- 最后针对修改的回调与错误恢复重新执行：37 项通过。
- motor_control_hybrid 与 humanoid_vision 顺序构建成功；`git diff --check` 通过。

通过日志分别为 `dds-fix-cpp-after.log`、`dds-fix-suite-v3.log`、
`dds-fix-regressions-final.log`。策略时间合同夹具补齐了已有启动参数，
没有放宽测试中的时效要求。

## 完整仿真与故障注入

全部为正常 GUI chase 启动、固定吊架、`start_chase:=false`、电机禁用、
`localhost_only:=0`，每次观察 32 秒。加载私有严格 DDS 的试验没有覆盖
系统库。观察器额外记录 C++ 故障状态和收到的 motor command 模式，因此
不能把本组时间直接与上组未订阅这些 topic 的基线作为单变量性能比较。

`publish_probe/libpublish_failure.so` 是独立的实验故障注入器，须显式设置
`HUMANOID_DDS_FAULT_INJECTION=held_sim` 及 topic 列表；仅用于此固定吊架
仿真。它让指定一次 rcl_publish 返回错误，不把失败伪装成成功，其余调用
真实转发。它不是正式启动的一部分，也不是 DDS 时间测量探针。

第一阶段保留四次记录：`dds-fix-natural-1/2`、`dds-fix-inject-sensors`、
`dds-fix-inject-commands`。natural-1 暴露原头部 tracker 的未捕获异常，
随后修复；两次 natural 均有 Safety 锁存，不能算健康控制链成功。
这阶段也发现采样失败日志的额外等待，最终版去掉采样路径日志。
不得用第一阶段的耗时评价最终版。

最终版 `dds-fix-final-inject` 确认七个错误点全部触发：
`/imu/data_raw`、`/joint_states`、`/safety/estop`、`/motor_commands`、
`/ball_chase/cmd_vel`、`/head/servo_target_rad`、`/head/tilt_target_rad`。
运行期间 **0 个节点异常退出**，IMU 持续到 t=32 s；C++ 报 FAILED，全程
收到 5598 条电机消息，全部为 DISABLE，没有自动恢复运动。
两轴 tracker 分别报告锁存停止输出。Safety 同时按原限值正常锁存。

`dds-fix-final-natural` 同样无提前退出，IMU 持续到观察结束，但仍在启动时
报 IMU age=255 ms 并锁存 Safety。这次开始约 1.23 秒有并行 Python 回归，
因此仅作为功能烟测，不据此作纯性能结论。

随后另做两次串行、无并行测试或编译的最终烟测：

| 启动 | 运行期间退出 | 最后 IMU | 结果 |
|---|---:|---:|---|
| 系统默认 DDS | 0 | t=32.00 s | IMU 发布最大间隙 241 ms，Safety 按源龄 250 ms 限值锁存；WBC 未开始 |
| 私有 strict + 5 ms | 0 | t=32.00 s | IMU 发布最大间隙 24.58 ms，Safety clear；chase 因 stale_chase_command 锁存，WBC 36 次后等待对齐输入 |

第二次的 IMU/Safety 健康也不能当成完整 chase 成功：策略已报告 stale
chase 命令故障。两次均保留在 `dds-fix-final-default` 和
`dds-fix-final-strict-clean` 下。所有本次试验都未释放吊架，没有行走成功率。

完整原始数据、所有失败和退出日志均保留在 `results/dds-fix-*`，自动汇总
见 [dds-fix-summary.json](results/dds-fix-summary.json)，可重新运行：

```bash
python3 experimental/reference_warp/analyze_publish_failures.py
```

**异常退出已有回归覆盖；启动阶段的 DDS/调度长延迟仍未消除。**
不能把“采样恢复、节点存活”写成“Safety 从此不超时”，更不能因此自动清除
故障继续走。严格实时私有库仍不自动启用。
