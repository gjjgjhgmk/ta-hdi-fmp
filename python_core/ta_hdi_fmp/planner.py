"""planner.py - TA-HDI-FMP 主入口

对应 MATLAB: run_single_trial.m

完整流程:
1. 离线准备：示教轨迹处理 + 模糊建模
2. 环境生成：障碍物生成 + 冲突检测
3. 局部重规划：每个冲突段进行 RRT* 重规划
4. 全局调制：FMP 调制 + push_repair + verify
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from .config import PlannerConfig, get_default_cfg
from .demo_processing import demo_processing, get_default_demo
from .fuzzy_model import fuzzy_model
from .geometry import generate_obstacles, detect_danger_segments, point_to_obstacle_distance
from .rrt_pool import get_informed_rrt_star_path_pool
from .hdi import adaptive_hdi
from .fmp_modulation import fuzregre_modulation_yout
from .push_repair import push_traj_clear
from .verify import verify_trajectory
from .score import score_path


def run_single_trial(
    cfg: Optional[PlannerConfig] = None,
    seed: Optional[int] = None,
    do_plot: bool = False
) -> Dict:
    """
    单次试验运行 TA-HDI-FMP

    Args:
        cfg: 配置参数
        seed: 随机种子
        do_plot: 是否绘图

    Returns:
        结果字典，包含 success, metrics, candidate_used, n_via
    """
    if cfg is None:
        cfg = get_default_cfg()

    if seed is None:
        seed = cfg.env_seed

    np.random.seed(seed)

    # ===== 阶段1: 离线准备 =====
    # 生成示教轨迹
    my_demos, x_line, y_line = get_default_demo(cfg.demo_len)
    demo_pos = np.array([x_line, y_line])

    # 数据预处理
    Data, demo_dura = demo_processing(
        my_demos, cfg.demo_len, cfg.demo_dt, cfg.alpha
    )

    # 模糊建模
    C_x, pInvCov_x, p1_u_x, C_y, pInvCov_y, p1_u_y, demo_dura = fuzzy_model(
        my_demos,
        cfg.demo_len,
        cfg.demo_dt,
        cfg.alpha,
        cfg.n_c,
        cfg.max_iter_fcm,
        cfg.max_iter_gk
    )

    # ===== 阶段2: 环境生成 =====
    obstacles = generate_obstacles(
        cfg.num_obs_target,
        cfg.env_seed,
        cfg.min_gap,
        x_range=(10, 90),
        y_range=(10, 90)
    )

    # 检测冲突段
    segments = detect_danger_segments(demo_pos, obstacles, cfg.safe_margin)

    # ===== 阶段3: 局部重规划 =====
    all_via_points = []
    all_via_times = []
    candidate_used = -1

    # 重规划输出时间参数
    timeinput = np.arange(
        cfg.dt_repro,
        cfg.demo_len * cfg.demo_dt,
        cfg.dt_repro
    )
    N_Data = len(timeinput)

    for seg in segments:
        idx_start = max(0, seg[0] - 4)
        idx_goal = min(cfg.demo_len - 1, seg[1] + 4)

        # RRT* 候选池
        ls = demo_pos[:, idx_start]
        lg = demo_pos[:, idx_goal]

        pool = get_informed_rrt_star_path_pool(
            ls, lg, obstacles, cfg
        )
        if len(pool) == 0:
            continue

        # 示教轨迹段
        demo_seg = demo_pos[:, idx_start:idx_goal + 1]

        # 评分排序
        scores = np.array([score_path(cand.path, demo_seg, obstacles, cfg) for cand in pool])
        ord_idx = np.argsort(scores)
        ord_idx = ord_idx[:min(cfg.k_candidates, len(ord_idx))]

        # 尝试每个候选
        selected = False
        for oi, cand_idx in enumerate(ord_idx):
            cand = pool[cand_idx]

            # HDI 插值
            dense, t_local, danger_mask = adaptive_hdi(
                cand.path, obstacles, cfg, idx_start, idx_goal, cfg.demo_dt
            )
            if dense.shape[1] < 2:
                continue

            # via 点时间缩放
            vscaled = (cfg.demo_len * cfg.demo_dt * demo_dura) * (
                t_local / (cfg.demo_len * cfg.demo_dt)
            )

            # FMP 调制（x 和 y 分别建模，分别调用）
            yx, _ = fuzregre_modulation_yout(
                timeinput, demo_dura, cfg.alpha, N_Data, cfg.n_c,
                C_x, pInvCov_x, p1_u_x,
                [0, 1], [2],  # Location_Y=[2] 表示 x 坐标在 p1_u 中的位置
                vscaled, dense, [0]  # dense (2, N) 包含 x,y, Location_V=[0] 选择 x
            )
            _, yy = fuzregre_modulation_yout(
                timeinput, demo_dura, cfg.alpha, N_Data, cfg.n_c,
                C_y, pInvCov_y, p1_u_y,
                [0, 1], [2],  # Location_Y=[2] 表示 y 坐标在 p1_u 中的位置
                vscaled, dense, [1]  # Location_V=[1] 选择 y
            )

            # Push repair
            if cfg.enable_push_repair:
                traj_raw = np.vstack([yx, yy])
                traj_fixed = push_traj_clear(traj_raw, obstacles, cfg)
                yx, yy = traj_fixed[0, :], traj_fixed[1, :]

            # 验证
            traj = np.vstack([yx, yy])
            ok, _ = verify_trajectory(traj, obstacles, cfg)

            if ok:
                all_via_points = dense
                all_via_times = t_local
                candidate_used = oi
                selected = True
                break

        # 如果没有通过验证的候选，选择评分最低的
        if not selected and len(ord_idx) > 0:
            cand = pool[ord_idx[0]]
            dense, t_local, _ = adaptive_hdi(
                cand.path, obstacles, cfg, idx_start, idx_goal, cfg.demo_dt
            )
            if dense.shape[1] >= 2:
                all_via_points = dense
                all_via_times = t_local
                if candidate_used < 0:
                    candidate_used = 0

    # ===== 阶段4: 全局调制 =====
    if all_via_points.shape[1] > 0:
        vscaled = (cfg.demo_len * cfg.demo_dt * demo_dura) * (
            all_via_times / (cfg.demo_len * cfg.demo_dt)
        )

        yx, _ = fuzregre_modulation_yout(
            timeinput, demo_dura, cfg.alpha, N_Data, cfg.n_c,
            C_x, pInvCov_x, p1_u_x,
            [0, 1], [2],
            vscaled, all_via_points, [0]
        )
        _, yy = fuzregre_modulation_yout(
            timeinput, demo_dura, cfg.alpha, N_Data, cfg.n_c,
            C_y, pInvCov_y, p1_u_y,
            [0, 1], [2],
            vscaled, all_via_points, [1]
        )
    else:
        # 没有重规划段，使用示教轨迹插值
        yx = np.interp(
            np.linspace(1, cfg.demo_len, N_Data),
            np.arange(cfg.demo_len),
            x_line
        )
        yy = np.interp(
            np.linspace(1, cfg.demo_len, N_Data),
            np.arange(cfg.demo_len),
            y_line
        )

    # 全局 Push repair
    if cfg.enable_push_repair:
        traj_raw = np.vstack([yx, yy])
        traj_fixed = push_traj_clear(traj_raw, obstacles, cfg)
        yx, yy = traj_fixed[0, :], traj_fixed[1, :]

    # 全局验证
    traj = np.vstack([yx, yy])
    ok, metrics = verify_trajectory(traj, obstacles, cfg)

    return {
        'seed': seed,
        'success': ok,
        'metrics': metrics,
        'candidate_used': candidate_used,
        'n_via': all_via_points.shape[1] if len(all_via_points) > 0 else 0,
        'trajectory': traj,
        'demo': demo_pos,
        'obstacles': obstacles
    }
