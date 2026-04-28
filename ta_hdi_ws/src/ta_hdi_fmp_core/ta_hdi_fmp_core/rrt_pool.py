from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class PathCandidate:
    path: np.ndarray
    cost: float
    t_found: float
    iter_found: int


def _in_collision_rrt(new_node: np.ndarray, obstacles: np.ndarray, inflation: float) -> bool:
    if obstacles is None or obstacles.shape[0] == 0:
        return False
    for k in range(obstacles.shape[0]):
        obs = obstacles[k]
        obs_type = int(obs[5])
        cx, cy = float(obs[0]), float(obs[1])
        if obs_type == 1:
            if np.linalg.norm(new_node - np.array([cx, cy])) <= (float(obs[2]) + inflation):
                return True
        else:
            th = float(obs[4])
            hw = float(obs[2]) / 2.0 + inflation
            hh = float(obs[3]) / 2.0 + inflation
            dx = new_node[0] - cx
            dy = new_node[1] - cy
            lx = dx * math.cos(-th) - dy * math.sin(-th)
            ly = dx * math.sin(-th) + dy * math.cos(-th)
            if abs(lx) <= hw and abs(ly) <= hh:
                return True
    return False


def get_informed_rrt_star_path_pool(start_pos, goal_pos, obstacles: np.ndarray, cfg) -> list[PathCandidate]:
    start = np.asarray(start_pos, dtype=float).reshape(2)
    goal = np.asarray(goal_pos, dtype=float).reshape(2)

    step_size = cfg.step_size
    r_neighbor = cfg.r_neighbor
    max_iter = cfg.max_iter_cap
    time_budget = cfg.time_budget_per_segment
    goal_sample_prob = cfg.goal_sample_prob
    q_tol = cfg.rrt_quality_tolerance

    c_min = float(np.linalg.norm(goal - start))
    if c_min < 1e-9:
        return [PathCandidate(path=np.column_stack([start, goal]), cost=0.0, t_found=0.0, iter_found=0)]

    x_center = (start + goal) / 2.0
    direction = (goal - start) / c_min
    angle = math.atan2(direction[1], direction[0])
    cmat = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]], dtype=float)

    # tree rows: [x, y, parent_idx, cost]
    tree = np.array([[start[0], start[1], 0.0, 0.0]], dtype=float)
    c_best = float('inf')

    x_min = min(start[0], goal[0]) - 20.0
    x_max = max(start[0], goal[0]) + 20.0
    y_min = min(start[1], goal[1]) - 20.0
    y_max = max(start[1], goal[1]) + 20.0

    pool: list[PathCandidate] = []
    t0 = time.perf_counter()

    for it in range(1, max_iter + 1):
        if (time.perf_counter() - t0) >= time_budget:
            break

        if c_best < float('inf'):
            val = max(c_best * c_best - c_min * c_min, 0.0)
            r1 = c_best / 2.0
            r2 = math.sqrt(val) / 2.0
            lmat = np.array([[r1, 0.0], [0.0, r2]], dtype=float)
            rr = math.sqrt(np.random.rand())
            theta = 2.0 * math.pi * np.random.rand()
            x_ball = np.array([rr * math.cos(theta), rr * math.sin(theta)], dtype=float)
            rand_node = (cmat @ lmat @ x_ball) + x_center
        else:
            if np.random.rand() < goal_sample_prob:
                rand_node = goal.copy()
            else:
                rand_node = np.array([
                    x_min + np.random.rand() * (x_max - x_min),
                    y_min + np.random.rand() * (y_max - y_min),
                ], dtype=float)

        d = np.linalg.norm(tree[:, :2] - rand_node.reshape(1, 2), axis=1)
        nearest_idx = int(np.argmin(d))
        nearest_node = tree[nearest_idx, :2]

        theta_step = math.atan2(rand_node[1] - nearest_node[1], rand_node[0] - nearest_node[0])
        new_node = nearest_node + np.array([step_size * math.cos(theta_step), step_size * math.sin(theta_step)], dtype=float)

        if _in_collision_rrt(new_node, obstacles, cfg.rrt_inflation):
            continue

        d_all = np.linalg.norm(tree[:, :2] - new_node.reshape(1, 2), axis=1)
        neighbor_indices = np.where(d_all <= r_neighbor)[0]

        best_parent = nearest_idx
        min_cost = float(tree[nearest_idx, 3] + np.linalg.norm(new_node - nearest_node))
        for idx in neighbor_indices:
            ctmp = float(tree[idx, 3] + d_all[idx])
            if ctmp < min_cost:
                best_parent = int(idx)
                min_cost = ctmp

        new_row = np.array([[new_node[0], new_node[1], float(best_parent), min_cost]], dtype=float)
        new_idx = tree.shape[0]
        tree = np.vstack([tree, new_row])

        for idx in neighbor_indices:
            c_via_new = min_cost + d_all[idx]
            if c_via_new < tree[idx, 3]:
                tree[idx, 2] = float(new_idx)
                tree[idx, 3] = float(c_via_new)

        if np.linalg.norm(new_node - goal) <= step_size:
            cost_to_goal = float(min_cost + np.linalg.norm(new_node - goal))
            if cost_to_goal < c_best:
                c_best = cost_to_goal

            if cost_to_goal <= c_best * q_tol:
                path_pts = [goal.copy()]
                curr = new_idx
                while curr > 0:
                    path_pts.append(tree[curr, :2].copy())
                    curr = int(tree[curr, 2])
                path_pts.append(start.copy())
                path = np.column_stack(path_pts[::-1])

                dup = False
                for p in pool:
                    if abs(p.cost - cost_to_goal) < 1e-3:
                        mid_old = p.path[:, p.path.shape[1] // 2]
                        mid_new = path[:, path.shape[1] // 2]
                        if np.linalg.norm(mid_old - mid_new) < 0.5:
                            dup = True
                            break
                if not dup:
                    pool.append(PathCandidate(path=path, cost=cost_to_goal, t_found=(time.perf_counter() - t0), iter_found=it))

    pool.sort(key=lambda x: x.cost)
    if len(pool) > cfg.k_candidates:
        pool = pool[: cfg.k_candidates]
    return pool
