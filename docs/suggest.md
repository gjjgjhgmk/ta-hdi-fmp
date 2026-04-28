# TA-HDI-FMP 框架改进建议

> 基于 2026-04-25 代码评测，按优先级排序。

---

## 高优先级（影响复现性和发表）

### S1 — 消除 `run_single_trial` 双实现分叉

**问题**：`run_single_trial.m`（独立文件）与 `main_core_framework.m` 内嵌的同名函数是两套独立实现，环境生成、FMP 模型训练、调制逻辑各自维护，行为一致性无保证。`test_framework.m` 测试的是外部文件版本，但 `main_core_framework.m` 运行的是内嵌版本——benchmark 数据对应哪套代码无法确定。

**修改方案**：
- 删除 `main_core_framework.m` 中的内嵌 `run_single_trial` 函数
- 改为直接调用外部 `run_single_trial.m`：
  ```matlab
  out = run_single_trial(cfg, cfg.seed, cfg.do_plot);
  ```
- 确保 `run_single_trial.m` 作为唯一实现，`main_core_framework.m` 仅保留配置和入口逻辑

**涉及文件**：`main_core_framework.m`、`run_single_trial.m`

---

### S2 — T6 风险指标改为最小障碍距离

**问题**：`score_path.m` 第 24 行：
```matlab
J_risk = 1 / (mean(ds) + 0.1);
```
使用**平均**障碍距离的倒数，导致一条大部分远离障碍物但局部极近的路径风险被低估，可能选出局部碰撞的次优路径，论文中"最优路径选择"的声明站不稳。

**修改方案**：
```matlab
% 改为最小距离（或混合）
J_risk = 1 / (min(ds) + 0.1);
% 或混合版本（保留平均特征同时惩罚局部风险）
J_risk = 0.7 / (min(ds) + 0.1) + 0.3 / (mean(ds) + 0.1);
```

**涉及文件**：`score_path.m` 第 24 行

---

## 中优先级（影响实验质量）

### S3 — 候选池时间预算提高

**问题**：`cfg.time_budget_per_segment = 0.3`（秒）对于复杂场景偏短，实际常常只返回 1 条候选，T6 打分排序退化为无意义操作，多候选选优的设计目的落空。

**修改方案**：
```matlab
cfg.time_budget_per_segment = 0.8;  % 原 0.3，提高以增加候选多样性
```
若运行时间敏感，可同时缩短 `max_iter_cap`（如从 8000 降至 5000）以平衡。

**涉及文件**：`main_core_framework.m` 的 `get_default_cfg()`

---

### S4 — T3 回归容差放宽

**问题**：`cfg.rrt_shape_mean_tol = 0.8`（米），对于坐标范围 0~100 的场景，两条路径的平均形状偏差很容易超过 0.8m，导致回归测试频繁误判为"框架退化"，掩盖真实问题。

**修改方案**：
```matlab
cfg.rrt_shape_mean_tol = 2.0;   % 原 0.8，放宽至场景尺度的 2%
cfg.rrt_shape_max_tol = 4.0;    % 原 1.5
```
同时在 `compare_paths_approx.m` 中修复冗余计算（`cost_rel` 和 `len_rel` 当前计算完全相同）：
```matlab
% 当前（冗余）
len_old = cost_old;
len_new = cost_new;
% 修改为真实路径长度（与代价含义相同时可删除冗余变量）
% 如后续 cost 与 length 含义分离，则分别计算
```

**涉及文件**：`main_core_framework.m` 的 `get_default_cfg()`、`compare_paths_approx.m`

---

### S5 — 候选池去重逻辑加强

**问题**：`get_Informed_RRT_Star_path_pool.m` 中去重条件：
```matlab
if abs(pool(pidx).cost - cost_to_goal) < 1e-9 && size(pool(pidx).path,2) == size(path,2)
```
只检查代价差值 + 点数，两条不同路径如果恰好同代价同点数会被误判重复，候选多样性降低。

**修改方案**：加入路径中间点坐标的粗略比对：
```matlab
if abs(pool(pidx).cost - cost_to_goal) < 1e-3
    mid_old = pool(pidx).path(:, round(end/2));
    mid_new = path(:, round(end/2));
    if norm(mid_old - mid_new) < 0.5
        dup = true; break;
    end
end
```

**涉及文件**：`get_Informed_RRT_Star_path_pool.m` 第 112-118 行

---

## 低优先级（细节修正）

### S6 — T4 曲率索引偏移修正

**问题**：`adaptive_hdi.m` 第 23 行：
```matlab
kk = min(max(p-1, 1), numel(kappa_all));
```
`compute_discrete_curvature` 输出的 `kappa_all` 长度为 `N-2`（内点），对应路径段 2 至 N-1。但此处 `p` 从 1 开始，`p-1=0` 时取 `kk=1`（第 1 个内点曲率），实际上第 1 段（p=1）没有对应的曲率，导致危险区判断的曲率值与路径段不严格对应。

**修改方案**：
```matlab
% p 是路径段索引（1 到 N-1），kappa_all 是内点曲率（1 到 N-2）
% 第 p 段对应的曲率取端点 p 和 p+1 的均值（若存在）
if p <= numel(kappa_all)
    kappa = kappa_all(p);
elseif p > 1 && (p-1) <= numel(kappa_all)
    kappa = kappa_all(p-1);
else
    kappa = 0;
end
```

**涉及文件**：`adaptive_hdi.m` 第 20-26 行

---

### S7 — T4 HDI 分母上限保护

**问题**：`adaptive_hdi.m` 第 28 行：
```matlab
interp_dist = cfg.base_interp_dist / (1 + cfg.hdi_k_risk*risk_norm + cfg.hdi_k_curvature*kappa_norm);
```
当 risk 和 kappa 均接近 1 时，分母约为 4.5，步长缩小至 `0.25/4.5 ≈ 0.056m`，局部段可能生成数百个插值点，导致后续调制计算量激增，同时命中 via 点上限裁剪使得危险区覆盖反而下降。

**修改方案**：已有 `min_interp_dist` 下限保护（第 29 行），此问题已被 `max(interp_dist, cfg.min_interp_dist)` 缓解。可额外记录告警：
```matlab
if interp_dist <= cfg.min_interp_dist * 1.1
    % (可选) 统计被下限截断的次数，用于调试
end
```
此项已有兜底，**优先级低**，可暂不修改。

---

### S8 — score_weights 标注为消融实验超参数

**问题**：`cfg.score_weights = [0.4, 0.2, 0.3, 0.1]` 对规划结果影响极大，是融合框架论文的关键变量，但当前没有注释说明。

**修改方案**：在 cfg 注释中明确标注：
```matlab
% T6 候选打分权重 [长度, 曲率, 风险, 偏离示教]
% !! 论文消融实验变量 !! 建议对比：
%   均匀权重 [0.25 0.25 0.25 0.25]
%   安全优先 [0.2  0.1  0.6  0.1]
%   示教优先 [0.2  0.1  0.2  0.5]
cfg.score_weights = [0.4, 0.2, 0.3, 0.1];
```

**涉及文件**：`main_core_framework.m` 的 `get_default_cfg()`

---

## 修改优先级汇总

| 编号 | 问题 | 优先级 | 预计工时 | 发表影响 |
|------|------|--------|---------|---------|
| S1 | 双实现分叉 | 🔴 高 | 30 min | 复现性 |
| S2 | T6 风险用均值非最小值 | 🔴 高 | 5 min | 选路质量 |
| S3 | 候选池时间预算 | 🟡 中 | 2 min | 实验质量 |
| S4 | T3 回归容差 + 冗余计算 | 🟡 中 | 15 min | 测试可信度 |
| S5 | 候选池去重逻辑 | 🟡 中 | 15 min | 多样性 |
| S6 | T4 曲率索引偏移 | 🟢 低 | 10 min | 精度 |
| S7 | T4 分母上限 | 🟢 低 | 已有兜底 | — |
| S8 | score_weights 注释 | 🟢 低 | 5 min | 论文规范 |
