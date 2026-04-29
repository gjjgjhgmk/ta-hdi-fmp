# TA-HDI-FMP 深度改进科研计划（codex）

## 0. 目标与路线决策
目标：在复杂障碍环境下，将当前 TA-HDI-FMP 从“可运行原型”提升为“可稳定复现 + 可迁移 ROS2 + 可发表”的完整方案。

先回答关键问题：**先在 MATLAB 改进，再迁移到 Python/ROS2**。  
理由：
1. 当前核心函数均为 MATLAB 实现，直接迁移会叠加算法风险与工程迁移风险。
2. 现有实现仍有硬性问题（例如调制函数容量硬编码），应先在 MATLAB 修正并验证。
3. MATLAB 适合快速做消融实验和参数扫描，能更快收敛算法方向。

---

## 1. 现有代码深度解析（结合关键变量）

## 1.1 `demo_processing.m`
文件：[demo_processing.m](/d:/01_Code/FMP/FMP/func/demo_processing.m:1)

作用：
1. 构造时间特征：`Data(1,:) = j`，`Data(2,:) = exp(alpha*j/demoLen)`。
2. 重采样示教轨迹并生成位置与速度差分。
3. 输出 `Data` 和 `demo_dura` 供 FMP 聚类建模。

可改进点：
1. `demoNum = 1` 写死，难以扩展多条示教。
2. 小样本增强方式是固定平移复制，可能引入非物理偏差。
3. `alpha` 缺少系统调参范围与解释。

## 1.2 `fuzzymodellingCandGK.m` 与 `fuzzy_clustering.m`
文件：[fuzzymodellingCandGK.m](/d:/01_Code/FMP/FMP/func/fuzzymodellingCandGK.m:1)、[fuzzy_clustering.m](/d:/01_Code/FMP/FMP/func/fuzzy_clustering.m:1)

作用：
1. FCM + GK 计算聚类中心与逆协方差 `pInvCov`。
2. 使用加权回归训练规则参数 `p1_u`。

可改进点：
1. `cov_i^(-1)` 和 `(X'WX)^(-1)` 显式求逆，数值稳定性风险较高。
2. 缺少正则化项，局部样本不足时容易病态。
3. 未显式给出规则置信度用于后续调制权重。

## 1.3 `fuzregre_modulation_yout.m`（核心调制）
文件：[fuzregre_modulation_yout.m](/d:/01_Code/FMP/FMP/func/fuzregre_modulation_yout.m:1)

作用：
1. 用 `via_time`、`via_point` 计算 via 点对规则的隶属度。
2. 对规则中心和回归截距执行 switch / add 调制。
3. 输出调制后的轨迹。

关键问题（高优先级）：
1. `pInvCov2=zeros(...,25)` 和 `p1_u2=zeros(...,25)` 是硬编码，当前 `N_C=40` 会有越界风险。
2. 没有显式 `dilation_factor` 参数，协方差膨胀仍停留在概念层。
3. `1./MD2` 无 `eps` 防护，可能数值爆炸。
4. via 点近似全硬约束，易出现过约束振荡。

## 1.4 `main_core_framework.m`
文件：[main_core_framework.m](/d:/01_Code/FMP/FMP/main_core_framework.m:1)

作用：
1. 检测危险段 `danger_indices`。
2. 局部截断 RRT* 探路。
3. 固定步长 HDI 插值（`interp_dist = 0.25`）。
4. via 点映射后执行 FMP 调制。

可改进点：
1. 截断策略为首解优先，缺少质量门控。
2. HDI 固定密度，不适应复杂局部几何。
3. 调制后无闭环验收（碰撞/曲率/jerk）。

---

## 2. 三步走战略

## 2.1 第一步：工程优化（算法改进）

### P0（1-2 周，先修硬问题）
1. 修复调制函数硬编码容量：
   - `pInvCov2=zeros(d1,d2,N_C + N_via2)`
   - `p1_u2=zeros(d1,d2,N_C + N_via2)`
2. 所有 `1./MD2` 改成 `1./max(MD2,eps)`。
3. 回归求解改为线性方程 + 正则化（`(A+lambda*I)\B`）。

### P1（2-4 周，提升质量与鲁棒性）
1. 风险自适应 HDI：
   - `interp_dist = base_dist / (1 + k1*risk + k2*curvature)`。
2. 预算内多解择优 TA-RRT*：
   - 每段保留 K 条可行解，按代价排序：  
   `J = wL*长度 + wC*曲率 + wR*障碍风险 + wD*偏离示教`
3. 软硬混合 via 约束：
   - hard via 保证安全；
   - soft via 保持意图和平滑。

### P2（4-6 周，形成核心创新）
1. 自适应协方差膨胀：
   - `pInvCov_eff = pInvCov * (1 + beta*exp(-d_obs/sigma))`
2. 调制后闭环验收：
   - 约束 `min_dist`、`kappa_max`、`jerk_rms`；
   - 不合格时仅重规划失败片段。

### 关键变量实验范围
1. `alpha`: `[0.03, 0.20]`
2. `N_C`: `[20, 40, 60]`
3. `interp_dist`: `[0.10, 0.40]`（自适应基值）
4. `dilation beta`: `[0.0, 1.5]`

---

## 2.2 第二步：算法移植（MATLAB -> ROS2 Python）

## 2.2.1 移植顺序
1. 先实现 Python 数值等价版（NumPy/SciPy）。
2. 再封装 ROS2 节点与接口。
3. 最后接入 MoveIt2/Gazebo 与实机。

## 2.2.2 包结构建议
ROS2 包：`ta_hdi_fmp_planner`

1. `fmp_core/`
   - `demo_processing.py`
   - `fuzzy_clustering.py`
   - `modulation.py`
2. `planner/`
   - `local_rrt_star.py`
   - `adaptive_hdi.py`
3. `ros_nodes/`
   - `intent_model_node.py`
   - `hybrid_planner_node.py`
4. `interfaces/`
   - `ViaPoints.msg`
   - `TrajectoryQuality.msg`

## 2.2.3 MoveIt2/Gazebo 仿真路径
1. 选择 UR5/Franka 的标准 MoveIt2 配置。
2. Gazebo 随机场景发布障碍体到 `PlanningScene`。
3. 规划节点输出笛卡尔轨迹，经 MoveIt2 时间参数化为关节轨迹。
4. 记录 `ros2 bag`：
   - 规划时间
   - 成功率
   - 最小障碍距离
   - jerk 与曲率

## 2.2.4 实机 Demo 路径
1. 接 `ros2_control`，限速 + 急停 + 虚拟墙。
2. 固定 3 套场景录制演示：
   - 稀疏障碍
   - 窄通道
   - 半动态障碍插入

---

## 2.3 第三步：学术产出（论文）

## 2.3.1 核心贡献（Contribution）
1. 提出风险自适应 TA-HDI-FMP 分层混合规划框架。
2. 提出基于 `pInvCov` 的自适应协方差膨胀调制机制。
3. 提出调制后闭环验收与局部回路策略，提升复杂环境鲁棒性。

## 2.3.2 对比与消融实验
对比组：
1. FMP（无避障）
2. Informed RRT*
3. 原 TA-HDI-FMP
4. 改进 RA-TA-HDI-FMP

消融组：
1. 去掉 HDI
2. 固定 dilation（无自适应）
3. 去掉闭环验收

指标：
1. 避障成功率
2. 平均/最大/P99 规划时间
3. 路径长度
4. 最小障碍距离
5. 曲率峰值与 jerk RMS
6. 相对示教轨迹 RMSE
7. CPU 占用与平均功耗

统计：
1. 每场景至少 30 次随机种子重复。
2. 报告均值±标准差与显著性检验。

---

## 3. 里程碑（12 周）
1. 第 1-2 周：P0 修复与回归测试。
2. 第 3-6 周：P1/P2 实现与消融。
3. 第 7-9 周：Python 等价移植与 ROS2 联调。
4. 第 10-11 周：Gazebo/MoveIt2 实验。
5. 第 12 周：实机 Demo 与论文初稿。

---

## 4. 执行优先级
1. 先修 `fuzregre_modulation_yout` 的容量与数值稳定性。
2. 再做风险自适应 HDI + 多解择优 TA-RRT*。
3. 最后上自适应协方差膨胀与 ROS2 迁移。

这条路径最稳、落地最快，也最容易形成高质量实验结果。
