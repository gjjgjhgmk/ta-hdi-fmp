from __future__ import annotations

import numpy as np

from .curvature import compute_discrete_curvature
from .geometry import min_point_to_obstacles


def score_path(rrt_path: np.ndarray, demo_seg: np.ndarray, obstacles: np.ndarray, cfg) -> float:
    p = np.asarray(rrt_path, dtype=float)
    if p.size == 0 or p.shape[1] < 2:
        return float('inf')

    w = np.asarray(cfg.score_weights, dtype=float).reshape(-1)

    length = float(np.sum(np.linalg.norm(np.diff(p, axis=1), axis=0)))
    straight = float(np.linalg.norm(p[:, 0] - p[:, -1]))
    j_len = length / max(straight, 1e-8)

    kappa = compute_discrete_curvature(p)
    if kappa.size == 0:
        j_curv = 0.0
    else:
        j_curv = float(np.percentile(kappa, 99))

    ds = np.array([min_point_to_obstacles(p[:, i], obstacles) for i in range(p.shape[1])], dtype=float)
    j_risk = 0.7 / (float(np.min(ds)) + 0.1) + 0.3 / (float(np.mean(ds)) + 0.1)

    if demo_seg is None or np.asarray(demo_seg).size == 0:
        j_dev = 0.0
    else:
        d = np.asarray(demo_seg, dtype=float)
        n = min(p.shape[1], d.shape[1])
        j_dev = float(np.mean(np.linalg.norm(p[:, :n] - d[:, :n], axis=0)))

    j = np.array([j_len, j_curv, j_risk, j_dev], dtype=float)
    j_norm = j / (j + 1.0)
    return float(np.sum(w * j_norm))
