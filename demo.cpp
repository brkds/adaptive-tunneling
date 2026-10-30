// ============================================================================
// demo.cpp —— adaptive_tunneling.h 的最小自检程序
//
// 用途：验证三件事
//   1. SelectivityEstimator 的 p̂ 计算正确；
//   2. TunnelingController 按 p̂ 阈值正确选择隧道深度；
//   3. MultiHopTunneling 的多跳 BFS 展开 + 去重 + 预算保护工作正常。
//
// 编译（任意支持 C++11 的编译器）：
//   g++ -std=c++11 -O2 demo.cpp -o demo && ./demo
// ============================================================================

#include "adaptive_tunneling.h"
#include <cstdio>
#include <cassert>
#include <cmath>

using namespace adaptive_tunneling;

int main() {
    // ---- 1. 过滤通过率估计 ----
    // 用报告 Low 档实测数量级验证：FilterSkips ≈ 320.69、Mean I/O ≈ 4.38
    // 期望 p̂ = 4.38 / (320.69 + 4.38) ≈ 0.0135
    SelectivityEstimator est;
    for (int i = 0; i < 320; ++i) est.record_skip();
    for (int i = 0; i < 4;   ++i) est.record_io();
    double p_hat = est.pass_rate();
    std::printf("[1] Low 档量级下 p̂ = %.4f（期望 ≈ 0.0135）\n", p_hat);
    assert(std::fabs(p_hat - 0.0135) < 0.01);

    // ---- 2. 深度决策 ----
    // 注：上面 p_hat = 4/324 ≈ 0.0123，落区间 [0.01, 0.05)，应返回 d=2。
    TunnelingController ctl;   // tau=0.05, tau_deep=0.01, d_max=3
    std::printf("[2] p̂=%.4f（整数近似档）-> d=%u（期望 2）\n",
                p_hat, ctl.decide_depth(p_hat));
    assert(ctl.decide_depth(p_hat) == 2);
    // Low 档真实 Selectivity 中位数 0.545% ≈ 0.00545 < 0.01，应返回 d=3。
    std::printf("[2] 强过滤 p̂=0.00545（Low 档真实值）-> d=%u（期望 3）\n",
                ctl.decide_depth(0.00545));
    assert(ctl.decide_depth(0.00545) == 3);
    std::printf("[2] 中过滤 p̂=0.03  -> d=%u（期望 2）\n",
                ctl.decide_depth(0.03));
    assert(ctl.decide_depth(0.03) == 2);
    std::printf("[2] 弱过滤 p̂=0.12  -> d=%u（期望 1）\n",
                ctl.decide_depth(0.12));
    assert(ctl.decide_depth(0.12) == 1);

    // ---- 3. 多跳展开 ----
    // 小图：0-1-2-3 链式，另加 1-4。
    std::vector<std::vector<uint32_t>> adj = {
        {1},       // 0
        {0, 2, 4}, // 1
        {1, 3},    // 2
        {2},       // 3
        {1},       // 4
    };
    MultiHopTunneling tunnel(adj);

    std::unordered_set<uint32_t> visited;
    visited.insert(0);
    auto r1 = tunnel.expand(0, 1, visited);   // 一跳：{1}
    std::printf("[3] 一跳展开收集 %zu 个邻居（期望 1：{1}）\n", r1.size());
    assert(r1.size() == 1);

    visited.clear(); visited.insert(0);
    auto r2 = tunnel.expand(0, 2, visited);   // 两跳：{1,2,4}
    std::printf("[3] 两跳展开收集 %zu 个邻居（期望 3：{1,2,4}）\n", r2.size());
    assert(r2.size() == 3);

    // 预算保护：budget=2 时两跳应被截断到 ≤2 个
    visited.clear(); visited.insert(0);
    auto r3 = tunnel.expand(0, 2, visited, /*budget=*/2);
    std::printf("[3] 预算=2 时两跳展开收集 %zu 个（期望 ≤2）\n", r3.size());
    assert(r3.size() <= 2);

    std::printf("\n全部自检通过：在线指标、深度决策、多跳展开（含去重与预算保护）均符合预期。\n");
    return 0;
}
