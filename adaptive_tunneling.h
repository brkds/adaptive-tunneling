// ============================================================================
// adaptive_tunneling.h
//
// Adaptive Tunneling Width —— 课程实验《基于 SSD 的过滤向量检索系统性能测试
// 与设计比较》任务三「优化方向」的最小可运行原型。
//
// 对应报告第 5 节：把 GateANN 固定的「一跳隧道展开」改为按运行时过滤通过率
// 动态调整展开深度，用于缓解强过滤（低 Selectivity）下 Recall 崩塌的问题。
//
// 【重要声明】
//   1. 本文件是独立原型，用于演示第 5.4 节在线指标与深度决策的核心逻辑，
//      并未接入 GateANN 原始系统（未修改 pipe_search.cpp / ssd_index.h），
//      也未经性能验证。它不能替代正式实验结论。
//   2. 所有命名与数据结构语义对齐 GateANN 源码，便于后续真正接入：
//        - tunneling_get_adj(id, nnbrs, nbrs)  —— ssd_index.h:374
//        - n_filter_skip / n_io_sent           —— pipe_search.cpp 的
//          filter_precheck_ 分支（第 251 行 n_ios++、第 342 行 n_filter_skip++）
//        - MAX_N_EDGES = 1024                  —— ssd_index_defs.h:7
//   3. 在线指标 p̂ = n_io_sent / (n_filter_skip + n_io_sent)，
//      语义是「过滤通过率（匹配密度）」，不是被过滤掉的比例。
// ============================================================================

#ifndef ADAPTIVE_TUNNELING_H
#define ADAPTIVE_TUNNELING_H

#include <cstdint>
#include <vector>
#include <unordered_set>

namespace adaptive_tunneling {

// ---------------------------------------------------------------------------
// 在线过滤通过率估计器。
//
// 每经过一次过滤检查，候选要么「通过并发盘」(io_sent)，要么「被拦下」(filter_skip)，
// 二者互斥且覆盖。用滑动窗口累计后即可估计匹配密度。
// ---------------------------------------------------------------------------
struct SelectivityEstimator {
    uint64_t n_io_sent     = 0;  // 通过过滤、真正发盘的候选数
    uint64_t n_filter_skip = 0;  // 被 check_filter 拦下的候选数

    // 过滤通过率 p̂ ∈ [0, 1]。分母为 0（查询刚开始）时返回 0，等价于「尚无信息」。
    double pass_rate() const {
        const uint64_t denom = n_io_sent + n_filter_skip;
        if (denom == 0) return 0.0;
        return static_cast<double>(n_io_sent) / static_cast<double>(denom);
    }

    // 调用点：候选通过过滤 → record_io()；候选被拦下 → record_skip()。
    void record_io()   { ++n_io_sent; }
    void record_skip() { ++n_filter_skip; }
};

// ---------------------------------------------------------------------------
// 隧道深度决策。
//
// 语义与报告 5.4 节一致：
//   - p̂ 高（匹配密度高、弱过滤）→ d = 1，维持基线；
//   - p̂ 低（匹配密度低、强过滤）→ 提高 d，让隧道一次展开更多跳，
//     以跨越连续不匹配区域、恢复图可达性。
//
// 阈值 τ 与最大深度 d_max 是新增可调常量（报告 5.4 节列出），
// 需通过对照实验标定，这里给的是「示例默认值」，不是经验证的有效值。
// ---------------------------------------------------------------------------
struct TunnelingController {
    double   tau       = 0.05;  // 过滤通过率阈值：低于它才加深隧道
    double   tau_deep  = 0.01;  // 更低阈值：进一步加深
    uint32_t d_max     = 3;     // 最大隧道深度
    uint32_t budget    = 0;     // 每查询隧道展开节点预算 B_tunnel（0 = 不限制）

    // 给定当前通过率 p̂，返回应采用的隧道深度 d ∈ [1, d_max]。
    uint32_t decide_depth(double p_hat) const {
        if (p_hat < tau_deep) return d_max;
        if (p_hat < tau)      return (d_max >= 2 ? 2 : 1);
        return 1;
    }
};

// ---------------------------------------------------------------------------
// 多跳 BFS 展开的骨架（伪接入点）。
//
// 原始 GateANN 只做一跳：对 retset[marker].id 调一次 tunneling_get_adj。
// 多跳版本以该节点为根，在 FullAdjIndex 上做 BFS，最多 hop 层，收集去重后的邻居。
// 这里用邻接表抽象代替真实的 full_adj_ 指针，便于独立编译演示。
// ---------------------------------------------------------------------------
class MultiHopTunneling {
public:
    // adj: 邻接表（对应 FullAdjIndex），adj[id] 是节点 id 的邻居列表。
    explicit MultiHopTunneling(const std::vector<std::vector<uint32_t>>& adj)
        : adj_(adj) {}

    // 以 root 为根，展开 hop 层邻域；用 visited 去重，返回收集到的邻居 id 列表。
    // budget 用于限制每查询展开总量，触顶即截断（对应报告 5.4 节的 B_tunnel）。
    std::vector<uint32_t> expand(uint32_t root, uint32_t hop,
                                 std::unordered_set<uint32_t>& visited,
                                 uint32_t budget = 0) const {
        std::vector<uint32_t> frontier{root};
        std::vector<uint32_t> collected;
        uint32_t expanded = 0;

        for (uint32_t h = 0; h < hop; ++h) {
            std::vector<uint32_t> next;
            for (uint32_t u : frontier) {
                if (u >= adj_.size()) continue;
                for (uint32_t v : adj_[u]) {
                    if (visited.count(v)) continue;   // 去重
                    visited.insert(v);
                    next.push_back(v);
                    collected.push_back(v);
                    ++expanded;
                    if (budget && expanded >= budget) {  // 预算保护：触顶截断
                        return collected;
                    }
                }
            }
            frontier.swap(next);
            if (frontier.empty()) break;
        }
        return collected;
    }

private:
    const std::vector<std::vector<uint32_t>>& adj_;
};

} // namespace adaptive_tunneling

#endif // ADAPTIVE_TUNNELING_H
