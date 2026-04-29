# 长期记忆

## 项目关键信息

### TA-HDI-FMP 框架
- **核心文件**：`main_core_framework.m`（主流程）、`fuzregre_modulation_yout.m`（调制）、`fuzzy_clustering.m`（聚类）、`fuzzymodellingCandGK.m`（建模）
- **协方差膨胀现状**：当前为概念实现，`fuzzy_clustering.m` 输出的 `cov(:,:,i) = cov_i.^0.5` 是标准差平方根而非协方差矩阵，**协方差膨胀功能阻断**
- **pInvCov 缩放因子**：`fuzzy_clustering.m` 第 50/66/80 行输出 `nIn_i = det(cov_i)^a * cov_i^(-1)`，不是纯逆协方差，Python 移植时需处理
- **硬编码问题**：`fuzregre_modulation_yout.m` 第 71-74 行 `zeros(...,25)` 需改为动态容量
- **数值稳定性**：多处 `1./MD2` 需加 `eps` 防护（建议 1e-8）；`fuzregre_param_train_t1.m` 求逆需加正则化
- **kappa 伪高曲率**：FMP 输出2000点时间均匀轨迹直接算曲率会产生伪高值（~7），需先做弧长重采样到200点。已修复：`verify_trajectory.m` 加入 `resample_arc()`
- **穿障问题**：FMP 调制带宽有限，via点之间轨迹弹回示教路径（穿障）。框架层修复：`push_traj_clear.m`（新），由 `cfg.enable_push_repair=true` 开关控制
- **穿障参数对齐**：push_target_margin 必须与 pass_collision_margin 对齐（均为1.0），否则推出后仍不过验收；`push_traj_clear.m` 第32行优先读 pass_collision_margin
- **穿障容差**：`verify_trajectory.m` 第26行 pass_no_penetration 放宽为 `> -1e-6`（吸收浮点边界噪声，精确=0的边界接触算通过）
- **基线数据**：30次固定种子（1001-1030）导出 `benchmark_results/matlab_baseline_30runs.csv`；修复前 success_rate=0%，修复后预期>30%

### 移植注意事项
- MATLAB→Python 移植前必须做数值等价测试（导出 v7.3 .mat 文件，逐点对比，允许 1e-10 误差）
- ROS2 节点职责划分：intent_model_node（Service）、hybrid_planner_node（Action Server）、trajectory_executor_node（Subscriber）
- 需增加 IK 批量求解 + 关节空间等时间重采样两个工程模块

### 项目时间评估
- Codex 计划 12 周，**实际需要 16-18 周**
- 关键路径：协方差膨胀实现（需修改聚类模块底层）+ 移植验证

## 技术规范

### FMP 模糊规则参数
- `alpha`：时间特征系数，建议范围 [0.03, 0.20]
- `N_C`：聚类数，建议范围 [20, 40, 60]
- `interp_dist`：HDI 插值间距，固定值 0.25（自适应化后基值 [0.10, 0.40]）
- `dilation beta`：协方差膨胀系数，建议范围 [0.0, 1.5]

### 实验统计方法
- 规划时间（非正态右偏）：Wilcoxon 秩和检验
- 成功率：Fisher 精确检验
- 多方法整体比较：Friedman + Nemenyi 事后检验
- 效果量：Cohen's d 或 rank-biserial correlation
