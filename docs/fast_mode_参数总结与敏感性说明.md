# TA-HDI-FMP 快速平滑版参数总结与敏感性说明

## 1. 当前固定参数（平滑版）

以下参数以当前代码为准（`main_core_framework.m` 与 `run_fast_mode_local.m` 一致）：

```matlab
% Fast topology + smooth modulation (当前固定组)
cfg.safe_margin = 1.0;
cfg.rrt_inflation = 1.0;
cfg.time_budget_per_segment = 0.05;
cfg.K_candidates = 1;
cfg.hdi_k_risk = 2.5;
cfg.push_smooth_sigma = 8.0;
cfg.pass_collision_margin = 1.0;
cfg.push_target_margin = 1.0;
```

## 2. 为什么“改一点参数”会有明显变化

从第一性原理看，这个链路是强耦合、非线性的：

`RRT 粗拓扑 -> HDI 加密 via -> FMP 调制 -> push 修复 -> verify 判定`

任何一个环节的参数变化，都会放大到后续环节，典型原因如下：

1. `safe_margin` 与 `rrt_inflation` 不是线性影响  
   在 `adaptive_hdi.m` 中，危险判定已使用：
   `danger_margin = safe_margin * rrt_inflation`。  
   所以从 `1.0*1.0` 改到 `1.8*1.8`，不是“加了 0.8”，而是危险边界从 `1.0` 变成 `3.24`，危险区面积会显著扩大，via 分布会整体外移、变密。

2. `hdi_k_risk` 控制的是插值密度“斜率”  
   该值越大，靠近危险区时插值间距缩得越快。  
   当你把 `hdi_k_risk` 拉高时，via 数量会快速增加，FMP 会被更强地拉向风险外侧，轨迹几何形态会明显变化。

3. `pass_collision_margin` 是“成功/失败”的硬阈值  
   验证不是连续评分，而是阈值判定（过/不过）。  
   即使轨迹仅差 0.1，也可能从 success 变成 fail。  
   所以调整该参数会直接改变成功率统计口径。

4. `push_target_margin` 与 `push_smooth_sigma` 会改变最终形状  
   `push_target_margin` 越大，推离障碍越强；  
   `push_smooth_sigma` 越大，曲线更平但可能牺牲局部贴合。  
   这两者叠加会改变 `kappa`、`jerk`、`path_length`。

5. `K_candidates=1` + RRT 早停让系统对单次拓扑更敏感  
   你现在追求极速，候选冗余几乎没有，某次粗拓扑质量的波动会直接反映到最终轨迹。

## 3. 这次“看起来突然变差”的直接原因

本次主要是阈值口径被改动导致的：

1. 一度把 `pass_collision_margin` 从 `1.0` 改到 `0.8`；  
2. 同时把 `push_target_margin` 也改到 `0.8`；  
3. 这会让“推离目标”和“验收标准”同时变化，导致视觉平滑感和成功判定都可能偏离你上一次结果。

恢复到当前固定组后，这个偏移已消除。

## 4. 后续调参建议（避免再次跑偏）

1. 每次只改 1 个参数，其他全部冻结。  
2. 优先顺序建议：  
   `hdi_k_risk` -> `push_smooth_sigma` -> `safe_margin/rrt_inflation` -> `pass_collision_margin`。  
3. `pass_collision_margin` 视为“验收口径”，不要频繁和算法参数一起改。  
4. 保留固定脚本 `run_fast_mode_local.m` 作为唯一实验入口，避免两个入口参数不一致。

---

结论：  
这组算法对“边界阈值 + 密度权重”高度敏感，变化不是线性累加，而是链式放大。当前固定参数组的目标是：在保证快速拓扑的前提下，优先获得稳定平滑的输出。

