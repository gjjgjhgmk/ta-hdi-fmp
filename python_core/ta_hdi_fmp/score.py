"""score.py - 路径评分

对应 MATLAB: func/score_path.m

综合评分：安全性 + 平滑性 + 示教轨迹偏差
"""

from __future__ import annotations

import numpy as np

from .curvature import compute_discrete_curvature
from .geometry import min_point_to_obstacles


def score_path(rrt_path: np.ndarray, demo_seg: np.ndarray, obstacles: np.ndarray, cfg) -> float:
    """
    对 RRT 候选路径进行评分

    Args:
        rrt_path: RRT 路径 (2, N)
        demo_seg: 示教轨迹段 (2, demo_len)
        obstacles: 障碍物数组 (M, 6)
        cfg: PlannerConfig

    Returns:
        score: 评分（越低越好）
    """
    p = np.asarray(rrt_path, dtype=float)
    if p.size == 0 or p.shape[1] < 2:
        return float('inf')

    w = np.asarray(cfg.score_weights, dtype=float).reshape(-1)

    # 1. 长度评分（归一化）
    path_len = float(np.sum(np.linalg.norm(np.diff(p, axis=1), axis=0)))
    if demo_seg is not None and np.asarray(demo_seg).size > 0:
        demo_len = float(np.sum(np.linalg.norm(np.diff(demo_seg, axis=1), axis=0)))
        j_len = path_len / max(demo_len, 1e-8)
    else:
        straight = float(np.linalg.norm(p[:, 0] - p[:, -1]))
        j_len = path_len / max(straight, 1e-8)

    # 2. 曲率评分
    kappa = compute_discrete_curvature(p)
    if kappa.size == 0:
        j_curv = 0.0
    else:
        j_curv = float(np.percentile(kappa, 99)) / cfg.kappa_pass_thresh

    # 3. 安全性评分（障碍物碰撞风险）
    ds = np.array([min_point_to_obstacles(p[:, i], obstacles) for i in range(p.shape[1])], dtype=float)
    j_risk = 0.7 / (float(np.min(ds)) + 0.1) + 0.3 / (float(np.mean(ds)) + 0.1)

    # 4. 示教轨迹偏差评分
    if demo_seg is None or np.asarray(demo_seg).size == 0:
        j_dev = 0.0
    else:
        d = np.asarray(demo_seg, dtype=float)
        n = min(p.shape[1], d.shape[1])
        j_dev = float(np.mean(np.linalg.norm(p[:, :n] - d[:, :n], axis=0)))

    j = np.array([j_len, j_curv, j_risk, j_dev], dtype=float)
    j_norm = j / (j + 1.0)
    return float(np.sum(w * j_norm))


def resample_path(path: np.ndarray, n: int) -> np.ndarray:
    """将路径重采样到 n 个点"""
    path = np.asarray(path, dtype=float)
    if path.shape[1] < 2:
        return np.repeat(path, n, axis=1)

    seg = np.linalg.norm(np.diff(path, axis=1), axis=0)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    if s[-1] < 1e-10:
        return np.repeat(path[:, [0]], n, axis=1)

    sq = np.linspace(0.0, s[-1], n)
    out = np.zeros((2, n), dtype=float)
    out[0, :] = np.interp(sq, s, path[0, :])
    out[1, :] = np.interp(sq, s, path[1, :])
    return out
