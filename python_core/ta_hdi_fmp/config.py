"""config.py - TA-HDI-FMP 配置参数

与 MATLAB run_single_trial.m get_default_cfg() 一致
"""

from dataclasses import dataclass, field
from typing import List


@dataclass
class PlannerConfig:
    # === 阶段1: 离线准备 ===
    demo_len: int = 200                    # 示教轨迹点数
    demo_dt: float = 0.1                   # 示教采样时间
    alpha: float = 0.1                     # 指数特征系数
    n_c: int = 40                         # 模糊聚类数
    max_iter_fcm: int = 30                # FCM 最大迭代
    max_iter_gk: int = 30                 # GK 最大迭代
    dt_repro: float = 0.01                # 重规划输出时间步

    # === 阶段2: 环境 ===
    num_obs_target: int = 40              # 目标障碍物数量
    min_gap: float = 1.0                  # 障碍物最小间距
    env_seed: int = 2025                  # 环境随机种子
    max_env_tries: int = 20000            # 生成障碍物最大尝试次数
    safe_margin: float = 1.5              # 安全裕量
    rrt_inflation: float = 1.2           # RRT 膨胀系数

    # === 阶段3: RRT* 候选池 ===
    step_size: float = 1.5                # RRT 步长
    r_neighbor: float = 4.0               # 近邻半径
    max_iter_cap: int = 8000              # 最大迭代次数
    time_budget_per_segment: float = 0.3  # 每段时间预算(秒)
    goal_sample_prob: float = 0.2         # 目标采样概率
    k_candidates: int = 3                # 候选路径数量
    rrt_quality_tolerance: float = 1.15   # RRT 质量容差

    # === HDI 自适应插值 ===
    base_interp_dist: float = 0.25         # 基础插值间距
    min_interp_dist: float = 0.05         # 最小插值间距
    max_interp_dist: float = 0.6          # 最大插值间距
    hdi_k_risk: float = 2.0             # 风险权重
    hdi_k_curvature: float = 1.5         # 曲率权重
    curvature_norm_ref: float = 1.0      # 曲率归一化参考值
    via_cap_factor: float = 2.0          # via 点容量因子

    # === T6 评分权重 [length, curvature, risk, demo_deviation] ===
    score_weights: List[float] = field(default_factory=lambda: [0.4, 0.2, 0.3, 0.1])

    # === T3 近似回归 ===
    rrt_cost_rel_tol: float = 0.03
    rrt_len_rel_tol: float = 0.03
    rrt_cmp_n: int = 200
    rrt_shape_mean_tol: float = 0.8
    rrt_shape_max_tol: float = 1.5

    # === 验证阈值 ===
    kappa_pass_thresh: float = 4.0       # 最大曲率阈值
    jerk_pass_thresh: float = 500.0       # Jerk RMS 阈值
    curvature_resample_n: int = 200      # 曲率计算重采样点数
    pass_collision_margin: float = 1.0    # 碰撞检测裕量

    # === Push repair 安全修复 ===
    enable_push_repair: bool = True
    push_target_margin: float = 2.0  # 大于 safe_margin 确保通过检测
    push_max_iters: int = 30
    push_smooth_sigma: float = 5.0


def get_default_cfg() -> PlannerConfig:
    """获取默认配置"""
    return PlannerConfig()
