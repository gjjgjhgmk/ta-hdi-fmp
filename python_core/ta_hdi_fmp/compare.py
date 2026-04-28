from __future__ import annotations

import numpy as np


def _path_cost(path: np.ndarray) -> float:
    p = np.asarray(path, dtype=float)
    if p.size == 0 or p.shape[1] < 2:
        return float('inf')
    return float(np.sum(np.linalg.norm(np.diff(p, axis=1), axis=0)))


def _resample_by_arclen(path: np.ndarray, n: int) -> np.ndarray:
    p = np.asarray(path, dtype=float)
    if p.shape[1] < 2:
        return np.repeat(p[:, [0]], n, axis=1)

    seg = np.linalg.norm(np.diff(p, axis=1), axis=0)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    if s[-1] < 1e-10:
        return np.repeat(p[:, [0]], n, axis=1)

    sq = np.linspace(0.0, s[-1], n)
    out = np.zeros((2, n), dtype=float)
    out[0, :] = np.interp(sq, s, p[0, :])
    out[1, :] = np.interp(sq, s, p[1, :])
    return out


def compare_paths_approx(path_old: np.ndarray, path_new: np.ndarray, cfg) -> tuple[bool, dict]:
    cost_old = _path_cost(path_old)
    cost_new = _path_cost(path_new)
    cost_rel = abs(cost_new - cost_old) / max(cost_old, 1e-10)

    res_old = _resample_by_arclen(path_old, cfg.rrt_cmp_n)
    res_new = _resample_by_arclen(path_new, cfg.rrt_cmp_n)
    d = np.linalg.norm(res_old - res_new, axis=0)
    mean_d = float(np.mean(d))
    max_d = float(np.max(d))

    ok = (
        cost_rel <= cfg.rrt_cost_rel_tol
        and mean_d <= cfg.rrt_shape_mean_tol
        and max_d <= cfg.rrt_shape_max_tol
    )

    details = {
        'cost_old': float(cost_old),
        'cost_new': float(cost_new),
        'cost_rel': float(cost_rel),
        'mean_point_dist': mean_d,
        'max_point_dist': max_d,
    }
    return bool(ok), details
