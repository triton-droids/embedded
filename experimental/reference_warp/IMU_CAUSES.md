# IMU 超时原因研究：资料、源码和本机验证

后续已完成真实转发的 Python/C++ 对照与 rcl/rmw 分层计时，见
[PUBLISH_BOUNDARY_RESULTS.md](PUBLISH_BOUNDARY_RESULTS.md)。其中捕获了
331.9 ms 的真实 IMU RMW 发布等待；具体内部资源尚未定位。

日期：2026-10-31。本次没有修改控制器、DDS 配置、系统权限或安全时限。

## 结论

目前最值得追查的是：**长时间发布调用发生时，rclpy 持有 GIL，可能把
一次原生调用的等待放大为同进程多个 Python 线程的停顿。**

“rclpy 发布调用持有 GIL、底层阻塞会拖住其他 Python 线程”已经通过安装
版本对应源码和独立故障注入实验验证。**真实 331 ms 发布调用内部究竟在
等待哪一个锁、队列、系统调度或传输资源，仍没有被定位。**

发布端停顿和接收端超时可能是不同故障，需要分别测量。改变 CPU 优先级
不能消除阻塞调用或被其他线程持有的锁。

## 版本与调用链

本机 dpkg 版本：

- `ros-jazzy-rclpy`: `7.1.9-1noble.20260126.174822`
- `ros-jazzy-rmw-fastrtps-cpp`: `8.4.3-1noble.20260126.173317`
- `ros-jazzy-fastrtps`: `2.14.5-2noble.20260121.180353`

实际安装的 `rclpy/publisher.py` 将消息交给 C++ 绑定。
[rclpy 7.1.9 源码](https://github.com/ros2/rclpy/blob/7.1.9/rclpy/src/rclpy/publisher.cpp#L113-L125)
在转换消息后调用 `rcl_publish()`，此段及绑定定义没有显式释放 GIL。
[pybind11 官方文档](https://pybind11.readthedocs.io/en/stable/advanced/misc.html#global-interpreter-lock-gil)
说明其默认不会隐式释放 GIL。

[rmw_fastrtps 8.4.3 源码](https://github.com/ros2/rmw_fastrtps/blob/8.4.3/rmw_fastrtps_shared_cpp/src/rmw_publish.cpp#L30-L64)
继续调用 Fast DDS 的 DataWriter 写入接口。因此实际发布路径是：

```text
Python publish
  → 消息转换，GIL 仍被持有
  → rcl_publish
  → rmw_fastrtps 发布
  → Fast DDS DataWriter 写入
```

MuJoCo 的 C 函数释放 GIL，并不能推导出其他 Python 原生扩展也释放 GIL。

## 独立的本机故障注入实验

在隔离的 ROS domain 225 中，只创建研究节点和 `/research/gil_probe`
发布器，另有每 1 ms 更新一次的普通 Python 心跳线程。没有仿真、控制器、
Safety、WBC 或执行器节点。

对照组使用原始安装的 rclpy 发布。注入组仅在该短命子进程中使用
`LD_PRELOAD` 替换 `rcl_publish`，检查 `PyGILState_Check()`，等待 300 ms，
然后返回成功。**注入组的底层发布是无操作测试桩，不会发送任何 DDS 消息。**
没有修改系统库、安装包或父进程环境。

| 条件 | Python publish 耗时 | 心跳最大间隔 | 原生调用处 GIL 检查 |
|---|---:|---:|---|
| 原始发布，无订阅者 | 0.120 ms | 1.269 ms | 未拦截 |
| 底层测试桩注入 300 ms 等待 | 300.587 ms | 301.572 ms | `gil_held=1` |

源码、测试程序、编译后的测试库和原始输出保存在
`results/gil-probe/`。测试库只供这个无消息交付实验使用，不能预加载到真实
ROS 控制进程。此实验说明的是**故障传播机制**，不是对正常 DDS 背压、网络
丢包或某个 Fast DDS bug 的复现。没有订阅者的对照也不是全栈性能测试。

这也限定了之前诊断执行器隔离测试的意义：`Event.wait()` 会让出 GIL，
所以拆执行器能通过那类回调排队测试；本次原生等待测试覆盖了另一种
不能靠同进程 Python 线程隔离解决的停顿。

## 常见原因及本项目证据

| 原因 | 一般机制 | 当前证据与判断 |
|---|---|---|
| DDS 发布等待 | 可靠传输、历史缓存/资源限制、内部同步等可能使写入等待 | 已测到 IMU publish 调用 331 ms；具体内部等待点未定位，优先查 |
| 原生调用持有 GIL | 一条线程进入长调用，其他 Python 线程无法继续执行 Python | 当前版本发布路径和独立注入实验均支持此机制，可能放大发布等待 |
| 执行器回调串行阻塞 | 诊断、日志或其他慢回调推迟 IMU 接收/检查 | 真实 ROS 差分测试已复现；分离诊断能解决这类排队，但不是全部停顿 |
| CPU/锁竞争与优先级反转 | 高优先级线程仍需等待低优先级持有的共享资源 | 存在 GIL、world.lock 和原生通信线程依赖；未证明其单独造成 331 ms |
| 内存、缺页、功耗与系统负载 | 造成尾部延迟，平均 CPU 利用率未必很高 | 一般可能；没有本机故障窗口的缺页、换页、降频因果证据 |
| QoS 或时间戳不兼容 | 收不到消息，或收到但因旧/重复/未来时间戳而拒绝 | 本项目有校验和计数；不能只看 topic Hz。当前长发布调用不是这类原因能直接解释的 |
| 物理/控制已经主动暂停 | 采样源不再更新，随后合理触发 freshness | 早期追球转向门限停止之后确实发生；必须与首次调度故障分开 |

Fast DDS [写入文档](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/dds_layer/publisher/dataWriter/publishingData.html#blocking-of-the-write-operation)
说明可靠写入在资源受限时可以等待。
[eProsima 发布模式说明](https://www.eprosima.com/developer-resources/performance/dds-asynchronous-vs-synchronous-publishing)
特别解释了异步模式也可能遇到历史缓存和资源压力。因此现有
`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` 不能当成“不阻塞”的保证。

文档中的某些例子讨论大消息、网络丢包；**IMU 是小消息，不能直接据此
宣称本机网络已拥塞或某个订阅者没有 ACK。** 大图像发布与 IMU 仍共享
模拟器进程/GIL，所以其他 topic 的长发布也值得测量，但尚未得到归因。

Fast DDS 2.14 的
[ReliabilityQosPolicy 文档](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/api_reference/dds_pim/core/policy/reliabilityqospolicy.html)
给出的 `max_blocking_time` 默认值为 100 ms。当前尚未读取运行中 DataWriter
的最终 QoS，也没有分解 331 ms；不能因为耗时接近某个整数倍就认定其
等于若干次 DDS 超时，更不能把这项资源等待限制当作整个 Python 调用的
实测总时长上界。

共享内存传输与 Data-sharing 是不同机制。
[官方区别说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/datasharing.html)
不能用于证明本程序启用了 Data-sharing；本次没有建立该配置证据。

ROS 2 [执行器设计资料](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Executors.rst)
也说明 OS 线程调度和回调执行管理是不同层次；其主要示例针对 rclcpp，
不能未经验证套用为当前 rclpy 的精确执行顺序。
ROS 2 [实时系统资料](https://design.ros2.org/articles/realtime_background.html)
列出了动态内存、缺页和无界同步等实时性问题；这些是候选原因，不是本机
已有根因证据。

## 为什么暂时优先查发布路径

现有 `priority-gui-1` 中，最大发布调用为 331.3 ms，GC 最大约 0.61 ms，
物理采样锁等待最大约 6.07 ms。这个观测窗口内，GC 和采样锁等待的已测
时长不足以单独解释那次长发布调用；不代表其他原生锁或其他运行已被排除。

另一条 `cpu-stack-gui-2` 中，Safety 报 IMU age 340 ms，而发布最大间隔
仅 25.4 ms，独立观察者最大源龄约 26.3 ms。它提示 Safety 进程自己的
接收/调度/发布路径也可能停顿，不能把所有故障归为传感器停发。

## 下一步最有区分度的验证

1. 在独立最小 ROS 发布/订阅链中，对“无订阅者、仅 Safety、加入 policy/WBC”
   串行对照；逐个改变订阅者，测量发布调用及各读者源龄，而不是同时改 QoS
   和中间件。现有正式安全契约保持不变。
2. 将 publish 调用分解到 rcl/rmw/DataWriter 写入边界，并区分线程在运行、
   锁等待还是被系统调度出 CPU。优先使用不会中断实时线程的低开销计时；
   之前造成崩溃的信号采样和不稳定原生分析不作为证据。
3. 对照独立 C++ IMU 发布/接收与当前 Python 路径。C++/独立进程可以消除
   与 Python 工作线程共享 GIL 的传播通道，但 DDS 本身仍可能等待，所以
   这项改造也必须测量，不能预先宣称解决。

本次只研究并验证机制，没有放宽超时、关闭可靠急停、替换正式发布器，
也没有把测试桩加载到仿真或硬件链中。所有独立测试进程已结束。
