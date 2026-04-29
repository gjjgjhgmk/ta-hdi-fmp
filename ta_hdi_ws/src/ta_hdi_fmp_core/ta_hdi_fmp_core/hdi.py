from __future__ import annotations

import math

import numpy as np

from .curvature import compute_discrete_curvature
from .geometry import min_point_to_obstacles


def adaptive_hdi(rrt_path: np.ndarray, obstacles: np.ndarray, cfg, idx_start: int, idx_goal: int, demo_dt: float):
    p = np.asarray(rrt_path, dtype=float)
    dense_path = []
    is_danger_mask = []

    if p.size == 0 or p.shape[1] < 2:
        return np.zeros((2, 0), dtype=float), np.zeros((0,), dtype=float), np.zeros((0,), dtype=bool)

    kappa_all = compute_discrete_curvature(p)

    for seg_idx in range(p.shape[1] - 1):
        pt1 = p[:, seg_idx]
        pt2 = p[:, seg_idx + 1]
        mid = 0.5 * (pt1 + pt2)
        dmin = min_point_to_obstacles(mid, obstacles)
        risk = 1.0 / (dmin + 0.1)
        risk_norm = risk / (risk + 1.0)

        if kappa_all.size == 0:
            kappa = 0.0
        else:
            kk = min(max(seg_idx, 0), kappa_all.size - 1)
            kappa = float(kappa_all[kk])

        kappa_norm = min(kappa / max(cfg.curvature_norm_ref, 1e-6), 1.0)

        interp_dist = cfg.base_interp_dist / (1.0 + cfg.hdi_k_risk * risk_norm + cfg.hdi_k_curvature * kappa_norm)
        interp_dist = min(max(interp_dist, cfg.min_interp_dist), cfg.max_interp_dist)

        ninterp = max(2, int(math.ceil(np.linalg.norm(pt2 - pt1) / interp_dist)))
        xx = np.linspace(pt1[0], pt2[0], ninterp)
        yy = np.linspace(pt1[1], pt2[1], ninterp)
        seg_pts = np.vstack([xx, yy])

        seg_danger = np.array([
            min_point_to_obstacles(seg_pts[:, q], obstacles) < cfg.safe_margin for q in range(ninterp)
        ], dtype=bool)

        if seg_idx < p.shape[1] - 2:
            dense_path.append(seg_pts[:, :-1])
            is_danger_mask.append(seg_danger[:-1])
        else:
            dense_path.append(seg_pts)
            is_danger_mask.append(seg_danger)

    dense = np.concatenate(dense_path, axis=1) if dense_path else np.zeros((2, 0), dtype=float)
    danger = np.concatenate(is_danger_mask) if is_danger_mask else np.zeros((0,), dtype=bool)

    n_via_cap = max(2, int(round(cfg.via_cap_factor * cfg.n_c)))
    if dense.shape[1] > n_via_cap:
        danger_idx = np.where(danger)[0]
        safe_idx = np.where(~danger)[0]
        keep = np.unique(np.concatenate(([0], danger_idx, [dense.shape[1] - 1]))).astype(int)
        rest_cap = n_via_cap - keep.size
        if rest_cap > 0 and safe_idx.size > 0:
            pick = np.round(np.linspace(0, safe_idx.size - 1, min(rest_cap, safe_idx.size))).astype(int)
            keep = np.unique(np.concatenate((keep, safe_idx[pick]))).astype(int)
        keep = np.sort(keep)
        dense = dense[:, keep]
        danger = danger[keep]

    if dense.shape[1] < 2:
        return dense, np.zeros((0,), dtype=float), danger

    t_start = idx_start * demo_dt
    t_goal = idx_goal * demo_dt
    dacc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=1), axis=0))))
    if dacc[-1] < 1e-10:
        return dense, np.zeros((0,), dtype=float), danger

    t_local = t_start + (dacc / dacc[-1]) * (t_goal - t_start)
    return dense, t_local, danger
