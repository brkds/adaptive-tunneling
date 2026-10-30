# Adaptive Tunneling Width —— 优化方向原型

课程实验《基于 SSD 的过滤向量检索系统性能测试与设计比较》任务三的**最小可运行原型**，
对应报告第 5 节「自适应隧道宽度（Adaptive Tunneling Width）」。

## 这是什么

把 GateANN 固定的「一跳隧道展开」改为按运行时**过滤通过率**动态调整展开深度，
目标是在强过滤（低 Selectivity）下恢复图可达性、缓解 Recall 崩塌。

核心逻辑三部分（见 `adaptive_tunneling.h`）：

| 组件 | 作用 | 对应报告 |
|---|---|---|
| `SelectivityEstimator` | 在线估计过滤通过率 `p̂ = n_io_sent/(n_filter_skip+n_io_sent)` | 5.4 节 |
| `TunnelingController` | 按 `p̂` 阈值选隧道深度 d ∈ [1, d_max] | 5.4 节 |
| `MultiHopTunneling` | 多跳 BFS 展开 + 去重 + 预算保护 `B_tunnel` | 5.4 节 |

## 编译与自检

```bash
g++ -std=c++11 -O2 demo.cpp -o demo && ./demo
```

预期输出「全部自检通过」，验证三件事：p̂ 计算、深度决策、多跳展开（含去重与预算截断）。

## 重要声明

- 本原型**独立编译**，未接入 GateANN 原系统（未改 `pipe_search.cpp` / `ssd_index.h`），
  也未经过性能验证，**不能替代正式实验结论**。
- 数据结构语义对齐 GateANN 源码，便于后续真正接入：
  - `tunneling_get_adj(id, nnbrs, nbrs)` —— `ssd_index.h:374`
  - `n_filter_skip` / `n_io_sent` —— `pipe_search.cpp` 的 `filter_precheck_` 分支
  - `MAX_N_EDGES = 1024` —— `ssd_index_defs.h:7`
- 阈值 `τ` 与最大深度 `d_max` 是**示例默认值**，需通过对照实验标定（报告 5.7 节给出了
  2×2 交叉验证计划），不是经验证的有效值。

## 文件

```
adaptive-tunneling/
├── adaptive_tunneling.h   核心逻辑（头文件）
├── demo.cpp               自检程序
└── README.md              本文件
```
