from __future__ import annotations

import numpy as np

from .curvature import compute_discrete_curvature
from .geometry import min_point_to_obstacles


def verify_trajectory(traj: np.ndarray, obstacles: np.ndarray, cfg) -> tuple[bool, dict]:
    metrics = {
        'min_dist': float('inf'),
        'kappa_max': float('inf'),
        'jerk_rms': float('inf'),
        'pass_collision': False,
        'pass_curvature': False,
        'pass_jerk': False,
        'path_length': float('inf'),
    }

    t = np.asarray(traj, dtype=float)
    if t.size == 0 or t.shape[1] < 2:
        return False, metrics

    for i in range(t.shape[1]):
        metrics['min_dist'] = min(metrics['min_dist'], min_point_to_obstacles(t[:, i], obstacles))
    metrics['pass_collision'] = metrics['min_dist'] > cfg.safe_margin

    kappa = compute_discrete_curvature(t)
    if kappa.size == 0:
        metrics['kappa_max'] = 0.0
    else:
        metrics['kappa_max'] = float(np.percentile(kappa, 99))
    metrics['pass_curvature'] = metrics['kappa_max'] < cfg.kappa_pass_thresh

    acc = np.diff(t, n=2, axis=1)
    if acc.shape[1] >= 2:
        jerk = np.diff(acc, n=1, axis=1)
        metrics['jerk_rms'] = float(np.sqrt(np.mean(np.sum(jerk * jerk, axis=0))))
    else:
        metrics['jerk_rms'] = 0.0
    metrics['pass_jerk'] = metrics['jerk_rms'] < cfg.jerk_pass_thresh

    metrics['path_length'] = float(np.sum(np.linalg.norm(np.diff(t, axis=1), axis=0)))
    passed = metrics['pass_collision'] and metrics['pass_curvature'] and metrics['pass_jerk']
    return bool(passed), metrics
