# 真实发布路径对照与分层计时结果

2026-10-31，当前 Ubuntu/i7-1165G7 主机；ROS Jazzy、rclpy 7.1.9、
rmw_fastrtps_cpp 8.4.3、Fast DDS 2.14.5、rclcpp 28.1.16。

## 结论

**真实运行中已捕捉到数百毫秒的原生 RMW 发布等待。** IMU 的一个调用在
`rcl_publish` 内耗时 332.000 ms，其中嵌套的 `rmw_publish` 耗时
331.901 ms，线程 CPU 时间仅 0.043 ms；观察到一次主动上下文切换、零次
抢占式切换、零次轻/重缺页，GIL 检查为持有。调用最终返回成功。

这已把该事件的大部分耗时定位到 RMW 发布路径，而不是 IMU 消息的 Python
转换或 MuJoCo 物理计算。主动切换支持内部阻塞等待的解释，但**具体哪个锁、
队列或资源仍未确定**；也不能把等待唤醒后的调度时间精确拆出来。

同进程其他 topic 的发布也会影响 IMU。另一个运行里，`/joint_states`
的 RMW 调用等待 173.548 ms，IMU 自己的发布调用最大仅约 3.45 ms，IMU
发布间隔却达到 192.863 ms。这与 rclpy 发布持 GIL 时其他 Python 执行器
无法继续工作的机制一致。

独立 C++ 头部控制节点的 RMW 发布也曾耗时 326.615 ms。因此 C++ 可以
消除与 Python 共享 GIL 的连带影响，**不能保证 DDS 发布本身不等待**。

## 方法与测量边界

所有工具在 `publish_probe/`，编译产物在 `generated/publish_probe/`。
目录含 `COLCON_IGNORE`，不会被普通机器人工作区构建自动纳入。

1. 最小链路：Python 与 C++ 各自发布相同固定大小的 `sensor_msgs/Imu`，
   Reliable/KeepLast(10)，100 Hz，2 s 发现等待后测量 8 s。另有 1 ms
   心跳线程。读者均为独立 C++ 进程，避免同时改变发布和接收语言。
2. 原生计时：`libpublish_boundary.so` 在独立子进程内经 `LD_PRELOAD`
   拦截公开的 `rcl_publish` 与 `rmw_publish`，解析真实函数并原样转发
   参数和返回值。记录单调时钟、线程 CPU 时间、PID/TID、topic、GIL 状态。
3. 等待计数：第二版 `libpublish_wait.so` 另用 `getrusage(RUSAGE_THREAD)`
   记录长调用中的主动/抢占式上下文切换和缺页计数。
4. 全栈：实际 experimental chase launch，保留 RGB-D、YOLO、policy、
   WBC、C++、Safety、Web 和当前优先级配置。所有运行保持吊架固定、腿电机
   禁用，不请求 ENABLE/release，也没有操作真实硬件。

这里**没有注入延迟、没有替换消息交付、没有更改安装库或 QoS**；它不同于
先前无交付的 GIL 测试桩。没有使用信号采样或原生堆栈展开。

每个线程前五次、每百次发布以及所有 >=2 ms 的调用会记录 CSV；等待版额外
记录 >=20 ms 的调用。计时器和日志会改变部分执行时序，不能把结果直接
当作未插桩的故障发生率。最小无订阅者对照另有未插桩记录，全部发布调用
仍低于 1 ms。嵌套 RMW 计时区间内部不做日志写入；外层 rcl 区间会包含
内层返回后的少量记录开销。

当前 `/proc/sys/kernel/sched_schedstats=0`，无法使用运行队列等待时长统计；
本次没有修改该系统配置。可见 cgroup 祖先 CPU 配额没有设置上限，累计
quota throttle 次数为零，未发现配额限流证据。

## 最小 Python/C++ 链路结果

每种插桩配置重复两次，第二轮倒转顺序。混合读者为 1 个 Reliable 与
2 个 BestEffort，接收 history depth=1；这是最小消费模型，不是完整 WBC
与 Safety 实现。慢读者为 Reliable/depth=1，回调内休眠 100 ms。

| 发布语言与配置 | 两次发布最大耗时 | 每次发布数 | 每次应用层接收数 |
|---|---:|---:|---|
| Python，无读者 | 0.271–0.290 ms | 800 | 无读者 |
| C++，无读者 | 0.177–0.179 ms | 800 | 无读者 |
| Python，三个混合读者 | 0.176–1.581 ms | 800 | 各 800 |
| C++，三个混合读者 | 0.192–0.229 ms | 800 | 各 800 |
| Python，慢可靠读者 | 0.263–0.291 ms | 800 | 80 |
| C++，慢可靠读者 | 0.141–0.151 ms | 800 | 80 |

未插桩的无读者对照：Python 最大 0.233 ms，C++ 最大 0.790 ms；各发
800 条。共 14 个正式最小链路试验，全部正常退出、各发 800 条。

慢读者接收间隔约 100 ms，但消费到的消息最大源龄约 17–20 ms。该配置下，
它主要少处理样本，没有制造出数百毫秒的发布等待。不能据此断言任何慢
读者都无影响，也不能反过来认定真实故障必然是慢读者未 ACK。

第一版真实转发 smoke 因误把 RMW 分派符号定位到 `librmw.so` 而以退出码
96 终止，保存在 `results/publish-matrix/smoke-cpp-reader/`，**无效，不计入
上述 14 个试验**。修正为 `librmw_implementation.so` 后，第二次 smoke
实际发/收 100/100 条，才开始正式队列。所有失败记录均保留。

## 全链路观测：所有结果均保留

五次观测各 32 s，自启动计时，均为固定吊架测试。前三次为普通边界计时，
后两次增加线程上下文切换/缺页计数。域 226–230，localhost 限制关闭，
串行运行；编译和其他测量不与这些观测并行。

| 记录 stem | IMU 最大发布间隔 | IMU Python 发布调用最大 | 最终 Safety / 控制状态 |
|---|---:|---:|---|
| `boundary-headless-1` | 192.863 ms | 3.453 ms | Safety clear；chase stale-command 停止；WBC 0 次求解 |
| `boundary-gui-1` | 40.338 ms | 3.586 ms | Safety clear；policy 活跃；WBC 2,615 次求解 |
| `boundary-gui-2` | 19.327 ms | 3.882 ms | Safety clear；policy 活跃；WBC 2,638 次求解 |
| `boundary-wait-headless-1` | 135.182 ms | 0.765 ms | Safety IMU age 335 ms；policy estop；WBC 0 次求解 |
| `boundary-wait-gui-1` | 350.887 ms | 341.397 ms | Safety clear；policy 活跃；WBC 2,565 次求解 |

这些不能计为走路成功或失败，也不能把 Safety clear 等同于控制链全程正常。
两条停止记录没有被排除。`boundary-wait-headless-1` 的接收端 335 ms 超时
尚未由已捕获的单个发布事件完整解释，不能宣称所有超时已找到共同根因。

## 具体长调用

下表为观测窗口内的真实原生 RMW 调用，所有返回码为成功：

| 运行与 topic | RMW 耗时 | 线程 CPU 时间 | GIL | 主动/抢占式切换 |
|---|---:|---:|---|---|
| headless-1 `/joint_states` | 173.548 ms | 0.040 ms | 持有 | 第一版未采集 |
| headless-1 `/policy/desired_motor_subset` | 186.853 ms | 0.044 ms | 持有 | 第一版未采集 |
| gui-1 `/head/servo_target_rad` | 326.615 ms | 0.048 ms | C++ 进程，无 CPython 符号 | 第一版未采集 |
| wait-headless-1 `/safety/status` | 90.546 ms | 0.037 ms | 持有 | 1 / 0 |
| wait-gui-1 `/head/tilt_target_rad` | 292.326 ms | 0.057 ms | 持有 | 1 / 0 |
| wait-gui-1 `/imu/data_raw` | 331.901 ms | 0.043 ms | 持有 | 1 / 0 |
| wait-gui-1 `/experimental/chase_status` | 343.798 ms | 0.045 ms | 持有 | 1 / 0 |

第二版这些长调用的轻/重缺页增量均为零。耗时几乎都落在嵌套 RMW 调用内，
CPU 时间极少，并发生主动切换，支持原生阻塞路径而非长时间计算。
这仍不能命名具体互斥锁，或把唤醒后的调度排队完全排除。

`wait-gui-1` 的 IMU RMW 等待发生于观察者 t=5.111–5.443 s。第一条观测到
的 Safety 诊断在 t=6.716 s，已为 clear。因此该运行的最终 clear 不证明
Safety 可以容忍持续运行阶段的 332 ms 断流。

`gui-1` 头部控制长发布对应实际 `humanoid_vision/track_node` C++ 可执行文件，
不是仅根据 `gil=-1` 推断其语言。跨进程同类等待说明根本发布阻塞不限于
Python；GIL 会扩大其在 Python 进程内的影响范围。

## 已定位与未定位

已定位：

- 长耗时确实可以发生在 `rmw_publish` 原生调用内，早于回到 Python。
- 某个非 IMU topic 的长发布足以构成同进程 IMU 调度停顿的传播通道。
- 相同小型 IMU 流的独立 Python/C++ 与少量/慢读者对照没有复现该长等待。
- 多种实际 topic 存在长发布，单纯给 IMU 提优先级或迁移语言不足以保证
  DDS 写入有界。诊断线程隔离仍然共享 Python GIL。

未定位：

- Fast DDS 写入内部究竟是互斥锁、缓存/资源、发现阶段同步，还是其他路径。
- 哪个具体读者/参与者触发等待；不能凭这些结果认定是可靠 ACK 背压。
- 另一类发布保持更新、Safety 却判旧的超时事件的完整内部时间线。
- 行走期间是否存在相同问题；本次全部未释放，没有硬件实时性结论。

下一层应在最小可复现的全栈启动条件中，继续定位 Fast DDS 写入内部等待，
并把关键控制线程与可阻塞发布解耦。正式安全、可靠急停和原始 acquisition
时间戳契约没有改动。本次不把某次观测正常当作修复。

## 原始记录与复现

- `results/publish-matrix/`：最小链路的消息数、源龄、发布耗时和日志。
- `results/boundary-*.imu.json`、`.cpu.json`、`.log`：五组全栈结果。
- `results/boundary-*-trace/*.csv`：嵌套原生计时。
- `results/boundary-wait-*-trace/*.wait.jsonl`：长调用上下文切换/缺页。
- `results/publish-boundary-summary.json`：汇总，启动/退出的窗口外事件单独保留。

```bash
source scripts/activate_rosenv.sh
cmake -S experimental/reference_warp/publish_probe \
  -B experimental/reference_warp/generated/publish_probe -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build experimental/reference_warp/generated/publish_probe -j1
python experimental/reference_warp/publish_probe/run_matrix.py \
  --output-root "$PWD/experimental/reference_warp/results/my-publish-matrix"
python experimental/reference_warp/stack_cpu_probe.py --gui false --domain 226 \
  --name my-boundary-headless \
  --preload experimental/reference_warp/generated/publish_probe/libpublish_wait.so \
  --trace-dir experimental/reference_warp/results/my-boundary-headless-trace
```

选择未占用的域和新结果名。脚本拒绝覆盖既有目录。不要使用旧的
`results/gil-probe/rcl_publish_stub.so` 做真实交付测量。
所有本次创建的测试/仿真进程已停止，系统配置未变；退出日志均保留。
