from dataclasses import dataclass, field
from typing import Sequence


@dataclass
class PlannerConfig:
    demo_len: int = 200
    demo_dt: float = 0.1
    alpha: float = 0.1
    n_c: int = 40
    max_iter_fcm: int = 30
    max_iter_gk: int = 30
    dt_repro: float = 0.01

    num_obs_target: int = 40
    min_gap: float = 1.0
    env_seed: int = 2025
    max_env_tries: int = 20000
    safe_margin: float = 0.5
    rrt_inflation: float = 1.2

    step_size: float = 1.5
    r_neighbor: float = 4.0
    max_iter_cap: int = 8000
    time_budget_per_segment: float = 5.0
    goal_sample_prob: float = 0.2
    k_candidates: int = 3
    rrt_quality_tolerance: float = 1.15

    rrt_cost_rel_tol: float = 0.03
    rrt_len_rel_tol: float = 0.03
    rrt_cmp_n: int = 200
    rrt_shape_mean_tol: float = 0.8
    rrt_shape_max_tol: float = 1.5

    base_interp_dist: float = 0.25
    min_interp_dist: float = 0.05
    max_interp_dist: float = 0.6
    hdi_k_risk: float = 2.0
    hdi_k_curvature: float = 1.5
    curvature_norm_ref: float = 1.0
    via_cap_factor: float = 2.0

    score_weights: Sequence[float] = field(default_factory=lambda: (0.4, 0.2, 0.3, 0.1))

    kappa_pass_thresh: float = 2.0
    jerk_pass_thresh: float = 500.0
