from __future__ import annotations

import math
from typing import Sequence

import numpy as np


# obs format: [cx, cy, p3, p4, theta, type], type=1 circle, type=2 OBB
def point_to_obstacle_distance(point: Sequence[float], obs: Sequence[float]) -> float:
    p = np.asarray(point, dtype=float).reshape(-1)
    cx, cy, p3, p4, theta, obs_type = [float(x) for x in obs]

    if int(obs_type) == 1:
        r = p3
        return float(np.linalg.norm([p[0] - cx, p[1] - cy]) - r)

    w = p3 / 2.0
    h = p4 / 2.0
    dx = p[0] - cx
    dy = p[1] - cy
    ct = math.cos(-theta)
    st = math.sin(-theta)
    lx = dx * ct - dy * st
    ly = dx * st + dy * ct
    qx = abs(lx) - w
    qy = abs(ly) - h
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
