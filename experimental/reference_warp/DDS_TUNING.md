# Fast DDS 调优顺序：当前版本的约束

2026-10-31，针对本机 Fast DDS 2.14.5 / rmw_fastrtps_cpp 8.4.3。
本次只检查源码、安装头文件和制定验证顺序，未修改正式配置或安装库。

后续已完成私有严格构建与六次对照，见
[DDS_TUNING_RESULTS.md](DDS_TUNING_RESULTS.md)。5 ms 改善 IMU 流，但仍有
其他 writer 超时导致 cpp_control_node 退出，尚不能作为默认修复。

## 先核实严格实时构建

已安装 `/opt/ros/jazzy/include/fastrtps/fastrtps/config.h:75` 定义
`HAVE_STRICT_REALTIME 0`。这是安装开发包的配置标记；尚未对发行版动态库
做逐指令的构建选项核验，也未建立新构建的运行对照。

[Fast DDS v2.14.5 写入源码](https://github.com/eProsima/Fast-DDS/blob/v2.14.5/src/cpp/fastdds/publisher/DataWriterImpl.cpp#L958-L981)
在取得 writer 互斥锁时根据该宏分支：严格模式使用有期限的尝试，非严格模式
使用普通锁获取。该分支**提供了超过配置时间仍等待的可能机制**，但尚未
证明前述 332 ms 事件正发生在这把锁上。

[官方 non-blocking 文档](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/use_cases/realtime/blocking.html)
要求严格实时构建与等待期限配置配合使用。因此，仅修改 XML 的
`max_blocking_time`，或仅给机器人节点编译时加一个宏，都不等于已经改造了
现有 Fast DDS 库的内部锁行为。

可复核的下一步是：在独立前缀/工作区构建相同版本 Fast DDS，启用
`-DSTRICT_REALTIME=ON`，使用匹配头文件和依赖重建所需 RMW/应用，明确核实
实际加载的动态库，再与原始安装做单变量对照。不要覆盖 `/opt/ros` 或在
同一进程混用两套不同构建的头文件和库。严格构建仍不等于硬实时保证。

## 参数候选：均需实测

1. **限制关键数据写入等待**：给 IMU、关节反馈等具体 writer 先试
   `max_blocking_time=5 ms`，作为 100 Hz IMU 的实验起点。它不是已验证值，
   也不是整个 Python 调用的绝对上界。对 `TIMEOUT` 处理必须配套：传感器帧
   失败时保留原始采样时间、记录计数、下一周期采集新帧；持续断流仍走既有
   watchdog。当前部分 Python 发布没有捕获该异常，直接改参数可能导致
   执行器退出，不能先在正式启动中盲目启用。急停必须单独保留可靠/锁存
   和明确的失败处理，不能套用“超时丢帧继续”的传感器规则。
2. **缩短历史并限制资源**：实时状态流以最新样本为目标，先单独试
   KeepLast 1–3，保持 reader/writer 的兼容性。资源上限与 history 深度必须
   一致；不要用任意增大队列掩盖旧消息积压。Safety 接收已经是 depth=1。
   2026-11-01 已将 IMU/关节反馈发布改为 Reliable/KeepLast=3；已有 depth=1
   的读端保持原值。此前实测报告中的 depth=10 是当时配置。调整前后都要
   确认实际配置；阻尼仍读取消息中的关节速度，不通过 DDS 队列差分。
3. **保留已知内存/发布模式**：现有启动已经是异步。当前 RMW 的常规策略
   包含可重分配的预分配历史和 Data-sharing OFF；这些已有设置不是新增
   修复。对 Image、String 等变长消息，不应直接套固定大小预分配。
4. **传输对照**：比较默认 SHM+UDP 与 UDP-only，保持相同 QoS、负载和网络
   发现范围；这只用于定位，不保证 UDP-only 更好。历史 UDP-only 观察没有
   消除命令过期问题。SHM transport 与 Data-sharing 必须分开讨论，不能
   把关闭其中一项误称为关闭两者。
5. **隔离大流与关键控制**：相机/点云的流量与消息发布不要运行在关键
   Python 控制线程内。需要评估 CPU、缓存及传输竞争；仅改 ROS_DOMAIN_ID
   不能提供 CPU 或物理网卡带宽的独占保障。

## XML 与 ROS QoS 的优先级

[rmw_fastrtps 8.4.3 配置说明](https://github.com/ros2/rmw_fastrtps/blob/8.4.3/README.md#full-qos-configuration)
指出，代码明确设置的 ROS QoS 通常优先于 XML 对应字段。该版本的 topic
profile 按 ROS 完整名称匹配，例如 `/imu/data_raw`。

`RMW_FASTRTPS_USE_QOS_FROM_XML=1` 会改变 memory policy、publication mode
及 Data-sharing 的取值来源；XML 没写全时可能回到不同默认值。它也会改变
`RMW_FASTRTPS_PUBLICATION_MODE` 的作用。若启用该开关，需明确写出并核实
异步发布、适合变长数据的 memory policy 和预期 Data-sharing 设置。
仅为调整不属于 ROS QoS 的 writer 等待参数，并不当然需要打开这个开关。

`ros2 topic info -v` 不会显示所有 Fast DDS 内部参数；应进一步读取真实
DataWriter QoS，确认等待时间、内存模式等已生效，不能只检查环境变量或
XML 能否解析。

## 验收

沿用 `publish_probe/` 的实际转发计时，串行对照原始/新配置，每次只改变一项。
同时包含启动发现阶段和稳定运行阶段，记录最坏 publish 时长、线程 CPU
时间、输入源龄、写入错误、丢帧、控制过期及 Safety 首次故障；保持控制周期、
超时、GUI/相机负载一致。不要只比较平均 Hz 或删掉启动失败样本。

先做严格实时构建核实与 writer QoS 回读，再做 5 ms 等待期限/异常处理的
对照，是目前比继续盲调队列更有针对性的顺序。内部具体锁的归因和运行
改善都仍需测量；本文件不是已完成调优的声明。
