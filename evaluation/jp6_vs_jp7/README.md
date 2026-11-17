# JetPack 6 与 JetPack 7 对比（双机拆分仿真）

在 JetPack 6 的 Jetson 上重复 JetPack 7 已经跑完的双机测试，参数和次数相同，
然后用 `compare.py` 两边并排对比。

| | JetPack 7（已完成） | JetPack 6（本目录） |
|---|---|---|
| 仿真机 | WSL Jazzy 发行版（`ssh -p 2222 rcli@192.168.1.129`，`~/ws/sim_jazzy`） | WSL Humble 发行版（`ssh rcli@192.168.1.129`，`~/ws/sim_experimental_cyclonedds`） |
| Jetson | `rcli@192.168.1.44`，L4T R39.2.1，链路 IP 192.168.50.2 | `td@100.71.186.19`，L4T R36.4.7，链路 IP 192.168.50.3 |
| Jetson 代码 | `~/Github/Humanoid_ros2_simulation` | `~/Documents/Github/embedded-jp6-split`（`jp6-baseline` worktree，基于 d92f5f73） |
| cuVSLAM | Isaac ROS 4.6（`humanoid_cuvslam:4.6-jp7`） | Isaac ROS 3.2（`humanoid_cuvslam:3.2`，`scripts/run_cuvslam_jp6.sh`） |
| 结果标签 | `base01`、`ab_vio01` … | `j6_base01`、`j6_ab_vio01` … |

两边都是 MAXN_SUPER，ROS_DOMAIN_ID 12，CycloneDDS 单播（`cyclonedds_split.xml`）。
两个发行版 ROS 版本不同，所以 JP6 必须配 Humble 发行版的仿真，不能混用。

## 测试组（全部 0.5 倍速，困难场景，16 段路线，A*，`odometry:=leg`）

| 组 | 控制回路 | 定位 | JP7 结果（摔倒） |
|---|---|---|---|
| `base` | Jetson（`sim`/`robot`） | 无 VIO，无 cuVSLAM | 0/10 |
| `ab_vio` | Jetson | cuVSLAM + IMU 融合 | 3/10 |
| `ab_vo` | Jetson | cuVSLAM 纯双目 VO | 0/10 |
| `c_vio` | WSL（`sim_control`/`perception`，CPU 0–3） | VIO | 0/9（c_vio09 作废） |
| `c_vo` | WSL | VO | 1/10 |

每轮约 6.5 分钟（JP7 实测），5 组 × 10 轮约 5.5 小时；`N=5` 约 2.7 小时。`batch.sh` 按轮交错跑，中途停也能对比。

## 明天的步骤

1. 网线插到 JP6 Jetson 的网口（`enP8p1s0`）。在 Jetson 上（需要 sudo 密码）：

   ```bash
   sudo bash ~/jetson_link_setup.sh
   ```

   建一个 NetworkManager 配置 `wsl-direct`：192.168.50.3/24，不设默认路由，Wi-Fi 和
   Tailscale 不受影响。撤销：`sudo bash ~/jetson_link_setup.sh remove`。
   WSL 一侧的 192.168.50.1 是 Windows 网卡上的设置，不用改。

2. 调度在 WSL 上跑（Humble 发行版，`ssh rcli@192.168.1.129`），笔记本可以合上。
   WSL 的公钥已加到 JP6 的 `td`，经直连链路 `td@192.168.50.3` 登录
   （WSL 访问不到 Tailscale 地址）。`hosts.sh` 检测到本机有 192.168.50.1 时自动用本地模式；
   在笔记本上运行时还是走 ssh（`WSL=rcli@192.168.1.129 JET=td@100.71.186.19`）。

   ```bash
   cd ~/ws/sim_experimental_cyclonedds/evaluation/jp6_vs_jp7
   bash preflight.sh
   ```

3. 先试一轮（约 7 分钟），看结果正常再跑全部：

   ```bash
   N=1 RUN_GROUPS=ab_vio bash batch.sh
   ```

   试跑占用了 `j6_ab_vio01`，正式批量从第 2 轮开始或者先删掉它
   （WSL 和 Jetson 的 `~/split_test/j6_ab_vio01`）。

4. 正式批量：

   ```bash
   setsid nohup bash batch.sh > ~/jp6_batch.log 2>&1 < /dev/null &
   ```

   跑完当前这轮后停：`touch /tmp/jp6_batch.stop`；续跑：`START=4 bash batch.sh`。
   WSL 不能关，Windows 也不能睡眠。

5. 对比（在笔记本上，要读 JP7 Jetson 192.168.1.44 的数据）：

   ```bash
   python3 evaluation/jp6_vs_jp7/compare.py          # 每组汇总
   python3 evaluation/jp6_vs_jp7/compare.py --runs   # 加上每一轮
   ```

## 文件

| 文件 | 在哪跑 | 作用 |
|---|---|---|
| `jetson_link_setup.sh` | JP6 Jetson，sudo | 链路 IP 和 rmem_max |
| `hosts.sh` | — | 判断在 WSL 本地还是笔记本上运行 |
| `preflight.sh` | WSL 或笔记本 | 两台机器、链路、编译、镜像、跨机 DDS 检查 |
| `run_one.sh` | WSL 或笔记本 | 跑一轮：先起 WSL 仿真，15 s 后起 Jetson，等结束 |
| `batch.sh` | WSL 或笔记本 | 5 组 × N 轮 |
| `split_wsl6.sh` | WSL 仓库内（Humble） | 仿真 + route_driver + 两个 localization_eval + RTF |
| `split_jet6.sh` | JP6 Jetson 仓库内 | 机器人栈 + cuVSLAM + tegrastats + disable_probe |
| `compare.py` | 笔记本 | JP6/JP7 对比表（连不上 JP7 时用 `jp7_reference.txt`） |
| `progress.py` | WSL | 实时进度：`watch -n 15 python3 progress.py` |
| `stamp_probe.py`、`diag_run.sh` | WSL | 跑一轮时在两端统计双目/IMU 时间戳间隔 |

`split_wsl6.sh`、`split_jet6.sh` 和 JP7 用的 `~/split_wsl.sh`、`~/split_jet.sh`
逻辑一样，只换了仓库路径、ROS 版本和链路 IP。每轮多写一个 `meta.txt`（提交、参数、
L4T 版本、功耗模式）。

## 与 JP7 的差别（对比时要记住）

- Isaac ROS 3.2 对 4.6：同一个 `cuvslam.yaml` / launch，3.2 读 `enable_imu_fusion`，
  4.6 读 `tracking_mode`，各自忽略另一个。
- 3.2 容器要挂载主机的 tegra 和 VPI 库，并且以 `--privileged` 运行（迁移前的做法）。
- 仿真机：WSL 里的 Humble 和 Jazzy 是两个发行版，硬件相同，但 ROS 版本和 Python
  不同。在 WSL 上测得的 RTF 差异也可能来自这里。
- JP6 的 rosenv 和 `.deps` 是链接到 `embedded-sim-experimental` 的旧环境，
  检测器 engine 从 `embedded-vio-imu` 复制（TensorRT 10.3，同一台设备上构建）。
