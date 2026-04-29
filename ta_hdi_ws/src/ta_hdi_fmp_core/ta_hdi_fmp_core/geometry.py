from __future__ import annotations

import math
from typing import Sequence

import numpy as np


# obs format: [cx, cy, p3, p4, theta, type] or [cx, cy, radius, w, h, theta, type]
# type=1 circle, type=2 OBB
def point_to_obstacle_distance(point: Sequence[float], obs: Sequence[float]) -> float:
    p = np.asarray(point, dtype=float).reshape(-1)
    obs_arr = np.asarray(obs, dtype=float)

    # 支持 6 或 7 元素格式
    if len(obs_arr) == 7:
        cx, cy, radius, w, h, theta, obs_type = obs_arr
    elif len(obs_arr) == 6:
        cx, cy, p3, p4, theta, obs_type = obs_arr
        radius = p3 if int(obs_type) == 1 else 0
        w = p3 if int(obs_type) == 2 else 0
        h = p4
    else:
        raise ValueError(f"Invalid obstacle format: expected 6 or 7 elements, got {len(obs_arr)}")

    obs_type = int(obs_type)

    if obs_type == 1:
        return float(np.linalg.norm([p[0] - cx, p[1] - cy]) - radius)

    w_half = w / 2.0
    h_half = h / 2.0
    dx = p[0] - cx
    dy = p[1] - cy
    ct = math.cos(-theta)
    st = math.sin(-theta)
    lx = dx * ct - dy * st
    ly = dx * st + dy * ct
    qx = abs(lx) - w_half
    qy = abs(ly) - h_half
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return float(outside + inside)


def min_point_to_obstacles(point: Sequence[float], obstacles: np.ndarray) -> float:
    if obstacles is None or len(obstacles) == 0:
        return float('inf')
    d_min = float('inf')
    for i in range(obstacles.shape[0]):
        d_min = min(d_min, point_to_obstacle_distance(point, obstacles[i]))
    return float(d_min)
