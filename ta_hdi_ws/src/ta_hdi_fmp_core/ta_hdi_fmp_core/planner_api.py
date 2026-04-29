from .config import PlannerConfig
from .rrt_pool import get_informed_rrt_star_path_pool
from .hdi import adaptive_hdi
from .score import score_path
from .verify import verify_trajectory

import numpy as np


def _path_to_metrics(path: np.ndarray, obstacles: np.ndarray, cfg: PlannerConfig):
    ok, metrics = verify_trajectory(path, obstacles, cfg)
    return bool(ok), metrics


def plan_path(start_xy, goal_xy, obstacles, cfg: PlannerConfig | None = None):
    cfg = cfg or PlannerConfig()
    obs = np.asarray(obstacles, dtype=float) if obstacles is not None else np.zeros((0, 6), dtype=float)

    pool = get_informed_rrt_star_path_pool(np.asarray(start_xy, dtype=float), np.asarray(goal_xy, dtype=float), obs, cfg)
    if len(pool) == 0:
        return {
            'success': False,
            'path_xy': np.zeros((0, 2), dtype=float),
            'metrics': {
                'min_dist': float('inf'),
                'kappa_max': float('inf'),
                'jerk_rms': float('inf'),
                'path_length': float('inf'),
            },
            'debug': {'candidate_used': -1, 'n_via': 0, 'status': 'NO_CANDIDATE'}
        }

    # v1: choose best by score on raw candidate paths (fast online baseline)
    best_idx = 0
    best_score = float('inf')
    demo_seg = pool[0].path
    for i, cand in enumerate(pool):
        s = score_path(cand.path, demo_seg, obs, cfg)
        if s < best_score:
            best_score = s
            best_idx = i

    best = pool[best_idx].path
    ok, metrics = _path_to_metrics(best, obs, cfg)

    return {
        'success': bool(ok),
        'path_xy': best.T,
        'metrics': {
            'min_dist': float(metrics['min_dist']),
            'kappa_max': float(metrics['kappa_max']),
            'jerk_rms': float(metrics['jerk_rms']),
            'path_length': float(metrics['path_length']),
        },
        'debug': {
            'candidate_used': int(best_idx),
            'n_via': int(best.shape[1]),
            'status': 'OK' if ok else 'VERIFY_FAIL'
        }
    }
