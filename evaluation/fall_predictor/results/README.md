# 跌倒预测器：训练与旁路测试结果（2026-11-07）

训练和在线旁路测试于 2026-11-07 在 `rcli@192.168.1.44`（`~/Github/Humanoid_simulation/embedded`，
分支 `sim-tuned-pd-fall-predictor`）上完成。该机器将重装 JetPack 7，结果在此留档。

本目录只收录报告、指标、划分和选中的 TorchScript 模型（约 1 MB）。原始采集数据
（`data/fall_collection`，1.8 GB）、全部 trial、逐样本预测和旁路日志（约 2.3 GB）不进 git；
完整副本保存在开发机 `rcli` 的仓库工作区 `data/fall_training`、`data/fall_collection`、
`local_test/fall_shadow/`（均被 `.gitignore` 忽略）。

**适用范围：** 数据来自旧的 RL tracking policy 控制路径（walking/standing reference，
tuned/original PD 或 WBC），同一 MuJoCo 仿真、同一控制器。不包含之后的 `controller:=model_walk`
（LIPM MPC + TSID），也没有实机数据。模型从未接入 Safety 或控制回路；分数未校准，不是跌倒概率。

## 训练

| 运行 | 模型 | 选择依据 | 关键结果 |
|---|---|---|---|
| [`20261107_v1`](20261107_v1/REPORT.md) | MLP 160→64→32→1，12,417 参数，300 ms | 验证 AP；验证窗口召回 ≥95% 定阈值 | 测试：事件 130/131，estop 前中位 150 ms，误报 390 次/小时 |
| [`auto_20261107`](auto_20261107/best.json) | 自动搜索 MLP/CNN，选中 trial 012（CNN，11,745 参数） | 验证集；阈值 0.95、连续 3 帧 | 验证：事件 119/124，estop 前 80 ms，误报 78 次/小时；[误报分析](auto_20261107/error_analysis/REPORT_ZH.md) |
| [`dual_v2_20261107`](dual_v2_20261107/training/REPORT.md) | 双时域 CNN（300/600 ms），选中 trial 03 | 新采集的恢复场景族；按族划分，封存测试只评一次 | 见下表 |

`dual_v2` 封存测试集（新场景族，43 次跌倒，6 个推后恢复）：

| 头 | 阈值 / 确认帧 | 事件检出 | estop 前中位 | 着地前中位 | 误报/小时 | 恢复被误报 | 窗口 AP |
|---|---|---|---|---|---|---|---|
| 300 ms | 0.602 / 3 | 36/40 | 160 ms | 278 ms | 116 | 0/6 | 0.932 |
| 600 ms | 0.99 / 5 | 43/43 | 250 ms | 360 ms | 24 | 0/6 | 0.876 |

测试集只有 60 个新 episode（6 个恢复），抽样不确定性大。

## 在线旁路测试（`shadow_node.py`，`dual_v2` trial 03）

节点只读 `/imu/data_raw` 并写日志，不发布、不影响 Safety。每行 `analysis/summary.json` 的
"estop 前" 为实时测得的中位数；误报为每负样本暴露小时的报警起始次数。

| 运行 | 有效 ep | 跌倒 / 恢复 / 无推 | 600 ms 检出 | 600 ms estop 前 | 600 ms 误报/h | 300 ms 检出 | 300 ms 误报/h |
|---|---|---|---|---|---|---|---|
| `shadow_50` | 49 | 16 / 1 / 32 | 16/16 | 295 ms | 19 | 16/16 | 37 |
| `shadow_push50` | 50 | 32 / 18 / 0 | 32/32 | 274 ms | 40 | 30/30 | 102 |
| `shadow_tuned50` | 50 | 3 / 2 / 45 | 3/3 | 175 ms | 0 | 3/3 | 8 |
| `shadow_tunedpush50` | 48 | 25 / 23 / 0 | 25/25 | 253 ms | 7 | 22/22 | 71 |
| `shadow_tunedskippush50` | 49 | 23 / 26 / 0 | 23/23 | 281 ms | 0 | 22/23 | 84 |
| `shadow_wbc50` | 47 | 35 / 12 / 0 | 35/35 | 252 ms | 41 | 33/33 | 138 |
| `shadow_wbccyc50` | 48 | 29 / 19 / 0 | 28/29 | 225 ms | 0 | 22/23 | 107 |
| `shadow_wbcfixed50` | 50 | 25 / 25 / 0 | 25/25 | 281 ms | 0 | 25/25 | 85 |
| `shadow_wbcfixednopush50` | 50 | 20 / 1 / 29 | 20/20 | 257 ms | 0 | 19/20 | 46 |
| `shadow_wbcgrace50` | 49 | 32 / 17 / 0 | 32/32 | 230 ms | 8 | 29/29 | 90 |

`*_smoke` 为 1–2 个 episode 的冒烟运行；`shadow_wbcfixed50_INVALID_oldpolicy` 用错了策略，已作废，
仅保留记录。`push50` 的在线推理 p50/p99 为 1.8/3.2 ms，偶发 136 ms 尖峰与 IMU 传输尖峰同时出现；
在线与离线特征最大偏差 4e-6。所有运行中推后恢复的 episode 都没有触发报警。

## 结论

在旧控制器下，600 ms 头能比现有确定性 estop 平均提前约 250 ms 给出预警，误报低；300 ms 头误报偏高。
要用于当前仿真栈，需要用 `model_walk` 重新采集、训练并重新做旁路测试；接入方式应为只告警的
advisory 节点，不参与 estop 锁存。

## 使用模型

`*/model_torchscript.pt` 接受 float32 `[batch, 20, 8]`（特征顺序见对应 `config.json`），归一化已内嵌，
输出 logit；dual 模型输出两个 logit（300 ms、600 ms）。先 sigmoid，再用 `best.json` 中对应头的
阈值和连续确认帧数。JSON 中的绝对路径指向原机器。
