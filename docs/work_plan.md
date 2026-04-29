# TA-HDI-FMP 混合框架完整实施方案（仅改框架层）

## 0. 约束与目标
### 0.1 约束（必须遵守）
1. 不修改 `func/` 下任何 FMP 底层函数。
2. 只修改混合框架与实验脚手架（核心文件为 [main_core_framework.m](d:/01_Code/FMP/FMP/main_core_framework.m)）。
3. 先完成 MATLAB 端可复现结果，再考虑 Python/ROS2 迁移。

### 0.2 项目目标
在复杂障碍环境下，把当前“可运行原型”升级为“可稳定复现、可做论文实验”的框架版 TA-HDI-FMP：
1. RRT 不再首解即停，改为预算内多解择优。
2. HDI 不再固定步长，改为风险/曲率自适应。
3. 增加调制后验收与回退，形成闭环。
4. 提供批量实验入口与结构化结果输出。

---

## 1. 参考意见整合（吸收与舍弃）
### 1.1 吸收的建议
1. 多候选路径 + 统一打分，不使用单一路径。
2. 增加离散曲率和最小障碍距离指标，避免只看路径长度。
3. via 点数量上限控制，避免过约束导致调制震荡。
4. 调制后增加碰撞/曲率/jerk 验收与回退重试。
5. 增加批量实验入口，输出 CSV 便于统计。

### 1.2 暂不采用的建议（当前阶段不做）
1. 修改 `fuzregre_modulation_yout.m` 内部逻辑（违反本阶段约束）。
2. 协方差膨胀底层实现（需改 `func/`，留到下一阶段）。
3. ROS2/Gazebo/实机执行链路（作为后续阶段）。

---

## 2. 总体技术方案（框架层）
### 2.1 现有流水线
感知 -> 局部 RRT -> HDI -> FMP 调制 -> 可视化

### 2.2 升级后流水线
感知 -> 分段局部 RRT（预算内多解） -> 候选路径打分 -> 自适应 HDI -> FMP 调制 -> 轨迹验收 -> 不通过则回退下一候选 -> 统计输出

---

## 3. 分阶段实施
## Phase A：框架可控化（第 1 周）
### A1 配置化参数（先做）
在 `main_core_framework.m` 头部加入统一 `cfg`：
1. `time_budget_per_segment`
2. `K_candidates`
3. `base_interp_dist`
4. `safe_margin`
5. `kappa_pass_thresh`
6. `jerk_pass_thresh`
7. `score_weights`
8. `via_cap_factor`

验收标准：
1. 代码中不再散落硬编码魔法数。
2. 改 `cfg` 可影响运行行为。

### A2 多解候选机制
改造局部规划函数为“时间预算 + 路径池”：
1. 每段收集最多 `K_candidates` 条可行路径。
2. 每条路径记录 `path/cost/time`。

验收标准：
1. 单段能输出多于 1 条候选（至少在部分随机种子下）。
2. 若只找到 1 条，流程仍可跑通。

---

## Phase B：路径质量提升（第 2 周）
### B1 候选路径打分与择优
代价函数：
`J = wL*length + wC*curvature + wR*risk + wD*deviation`

定义：
1. `length`：路径总长归一化。
2. `curvature`：离散曲率的 p95/p99。
3. `risk`：路径点到最近障碍距离的倒数均值。
4. `deviation`：相对名义示教段的几何偏差。

验收标准：
1. 候选能按分数排序。
2. 输出选择了第几条候选路径。

### B2 自适应 HDI
把固定 `interp_dist=0.25` 改为：
`interp_dist = base / (1 + k1*risk + k2*curvature)`

补充保护：
1. `interp_dist` 下限（避免爆量插值）。
2. `N_via_max = N_C * via_cap_factor`。
3. 超上限时做降采样（优先保留关键区）。

验收标准：
1. 危险区 via 点明显更密。
2. 总 via 点数受控不爆炸。

---

## Phase C：闭环稳定性（第 3 周）
### C1 调制后验收
验收项：
1. `min_dist`：最小障碍距离是否大于 `safe_margin`。
2. `kappa_max`：曲率上限是否达标。
3. `jerk_rms`：离散 jerk 是否达标。

### C2 回退机制
策略：
1. 首选分数最优候选路径调制。
2. 验收失败则自动切换下一候选。
3. 全失败时保底输出名义轨迹或首候选。

验收标准：
1. 能看到候选切换日志。
2. 全流程不因单候选失败崩溃。

---

## Phase D：实验化与论文数据（第 4 周）
### D1 批量实验入口
增加 `run_batch_trials(cfg)`：
1. 随机种子批量运行（建议 30）。
2. 每次记录核心指标。

### D2 结果落盘
输出 `benchmark_results.csv`，建议字段：
1. `seed`
2. `success`
3. `plan_time_total`
4. `plan_time_rrt`
5. `plan_time_modulation`
6. `path_length`
7. `min_dist`
8. `kappa_p99`
9. `jerk_rms`
10. `candidate_used`
11. `n_via`

验收标准：
1. CSV 可直接用于统计分析。
2. 同种子重复运行结果可复现。

---

## 4. 文件改动清单（本阶段）
1. [main_core_framework.m](d:/01_Code/FMP/FMP/main_core_framework.m)
2. 可选新增（如果脚本过长）：
   - `framework_helpers/score_path.m`
   - `framework_helpers/adaptive_hdi.m`
   - `framework_helpers/verify_trajectory.m`
   - `framework_helpers/run_batch_trials.m`

说明：若优先快速推进，可先全部写在 `main_core_framework.m` 的本地函数区，后续再拆分。

---

## 5. 里程碑与交付物
### M1（第 1 周末）
1. 配置化 + 多候选路径池可运行版本。
2. 单次可视化图可显示被选择的候选信息。

### M2（第 2 周末）
1. 自适应 HDI + 候选打分择优版本。
2. 对比固定 HDI 的初步效果图。

### M3（第 3 周末）
1. 验收与回退闭环版本。
2. 失败场景下自动回退日志。

### M4（第 4 周末）
1. 30 seeds 批量实验 CSV。
2. 初步统计摘要（成功率、耗时、路径质量）。

---

## 6. 风险与应对
### 风险 1：多候选路径收集不稳定
应对：
1. 提高时间预算。
2. 降低 `K_candidates`。
3. 允许多随机种子补充候选。

### 风险 2：自适应 HDI 导致 via 点过多
应对：
1. 插值最小步长下限。
2. via 上限与关键点优先保留。

### 风险 3：回退频繁导致耗时变大
应对：
1. 先用代价函数过滤明显差候选。
2. 回退最多尝试 `K` 次后保底退出。

---

## 7. 科研技能使用建议（已安装技能）
1. `matlab`：主实现与仿真。
2. `scientific-critical-thinking`：指标与实验设计审查。
3. `paper-lookup` / `research-lookup`：相关工作与对比方法检索。
4. `scientific-writing`：论文方法与实验章节草稿。
5. `sympy`：推导打分函数归一化或约束表达。

---

## 8. 第一周可直接执行清单
1. 在 `main_core_framework.m` 增加 `cfg` 区块。
2. 改造局部 RRT 函数返回候选路径池。
3. 增加 `score_path` 与候选排序。
4. 保持 FMP 调用接口不变。
5. 输出“候选编号 + 分数 + 是否通过验收”的日志。

---

## 9. 最终说明
本方案严格遵守你当前边界：先改“RRT+FMP 结合框架”，不动 FMP 底层。  
先把框架闭环和实验复现做稳，再进入底层协方差膨胀与 ROS2 迁移阶段。
