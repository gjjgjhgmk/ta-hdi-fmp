"""geometry.py - 几何计算工具

包括:
- 点到障碍物距离计算
- 障碍物生成
- signed_dist_and_grad: 带符号距离和梯度（用于 push_repair）
"""

from __future__ import annotations
import math
from typing import Sequence, Tuple, List

import numpy as np


# obs format: [cx, cy, p3, p4, theta, type], type=1 circle, type=2 OBB
def point_to_obstacle_distance(point: Sequence[float], obs: Sequence[float]) -> float:
    """
    计算点到障碍物的最小距离（欧几里得距离 - 障碍物半径/边界）

    Args:
        point: 点坐标 (2,) 或 (2, N)
        obs: 障碍物参数 [cx, cy, r/w, h, theta, type]
             type=1: 圆形 (r)
             type=2: OBB 矩形 (w, h, theta)

    Returns:
        最小距离（正值表示在外部，负值表示穿透）
    """
    p = np.asarray(point, dtype=float).reshape(-1)
    cx, cy, p3, p4, theta, obs_type = [float(x) for x in obs]

    if int(obs_type) == 1:
        # 圆形
        r = p3
        return float(np.linalg.norm([p[0] - cx, p[1] - cy]) - r)

    # OBB 矩形
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
    """
    计算点到所有障碍物的最小距离

    Args:
        point: 点坐标 (2,)
        obstacles: 障碍物数组 (M, 6)

    Returns:
        最小距离
    """
    if obstacles is None or len(obstacles) == 0:
        return float('inf')
    d_min = float('inf')
    for i in range(obstacles.shape[0]):
        d_min = min(d_min, point_to_obstacle_distance(point, obstacles[i]))
    return float(d_min)


def signed_dist_and_grad(pt: np.ndarray, obs: np.ndarray) -> Tuple[float, np.ndarray]:
    """
    计算点到障碍物的带符号距离和向外梯度

    用于 push_repair 模块

    Args:
        pt: 点坐标 (2,)
        obs: 障碍物参数 [cx, cy, r/w, h, theta, type]

    Returns:
        d: 带符号距离（正=外部，负=内部）
        grad: 向外的单位梯度向量 (2,)
    """
    cx = obs[0]
    cy = obs[1]
    typ = int(obs[5])

    if typ == 1:
        # 圆形
        r = obs[2]
        diff_v = pt - np.array([cx, cy])
        dist_c = np.linalg.norm(diff_v)
        d = dist_c - r
        if dist_c < 1e-10:
            grad = np.array([0.0, 1.0])
        else:
            grad = diff_v / dist_c
    else:
        # OBB 矩形
        w = obs[2]
        h = obs[3]
        theta = obs[4]
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        local = R.T @ (pt - np.array([cx, cy]))
        hw = w / 2
        hh = h / 2

        # 限制到矩形边界
        clamped = np.array([
            max(-hw, min(hw, local[0])),
            max(-hh, min(hh, local[1]))
        ])
        diff_l = local - clamped
        dl = np.linalg.norm(diff_l)

        if dl > 1e-10:
            # 外部：标准距离 + 梯度
            d = dl
            grad = R @ (diff_l / dl)
        else:
            # 内部：穿透深度
            pen_x = hw - abs(local[0])
            pen_y = hh - abs(local[1])
            if pen_x < pen_y:
                d = -pen_x
                grad = R @ np.array([1.0 if local[0] > 0 else -1.0, 0.0])
            else:
                d = -pen_y
                grad = R @ np.array([0.0, 1.0 if local[1] > 0 else -1.0])

    return float(d), grad


def generate_obstacles(
    num_obs: int,
    env_seed: int,
    min_gap: float = 1.0,
    x_range: Tuple[float, float] = (10, 90),
    y_range: Tuple[float, float] = (10, 90)
) -> np.ndarray:
    """
    生成随机障碍物环境

    对应 MATLAB: run_single_trial.m 障碍物生成部分

    Args:
        num_obs: 目标障碍物数量
        env_seed: 随机种子
        min_gap: 障碍物最小间距
        x_range, y_range: 生成范围

    Returns:
        obstacles: (num_obs, 6) 矩阵 [cx, cy, r/w, h, theta, type]
    """
    np.random.seed(env_seed)
    obstacles: List[List[float]] = []
    tries = 0
    count = 0

    while count < num_obs and tries < 20000:
        tries += 1
        ox = x_range[0] + (x_range[1] - x_range[0]) * np.random.rand()
        oy = y_range[0] + (y_range[1] - y_range[0]) * np.random.rand()
        typ = np.random.randint(1, 3)

        if typ == 1:
            # 圆形
            r = 2 + 3 * np.random.rand()
            cr = r
            newo: List[float] = [ox, oy, r, 0, 0, 1]
        else:
            # OBB 矩形
            w = 4 + 6 * np.random.rand()
            hh = 2 + 5 * np.random.rand()
            th = np.random.rand() * np.pi
            cr = np.sqrt((w / 2) ** 2 + (hh / 2) ** 2)
            newo = [ox, oy, w, hh, th, 2]

        # 检查与现有障碍物的间距
        valid = True
        for obs in obstacles:
            if obs[5] == 1:
                er = obs[2]
            else:
                er = np.sqrt((obs[2] / 2) ** 2 + (obs[3] / 2) ** 2)
            if np.linalg.norm([ox - obs[0], oy - obs[1]]) < (cr + er + min_gap):
                valid = False
                break

        if valid:
            obstacles.append(newo)
            count += 1

    return np.array(obstacles) if obstacles else np.zeros((0, 6))


def detect_danger_segments(
    demo_pos: np.ndarray,
    obstacles: np.ndarray,
    safe_margin: float = 1.5
) -> List[Tuple[int, int]]:
    """
    检测与障碍物冲突的示教轨迹段

    对应 MATLAB: run_single_trial.m danger 检测

    Args:
        demo_pos: 示教轨迹 (2, demo_len)
        obstacles: 障碍物数组 (M, 6)
        safe_margin: 安全裕量

    Returns:
        segments: 冲突段列表，每个元素为 (start_idx, end_idx)
    """
    demo_len = demo_pos.shape[1]
    danger: List[int] = []

    for k in range(obstacles.shape[0]):
        for j in range(demo_len):
            p = demo_pos[:, j]
            dist = point_to_obstacle_distance(p, obstacles[k, :])
            if dist < safe_margin:
                danger.append(j)

    danger = sorted(set(danger))

    if not danger:
        return []

    # 合并相邻冲突点为段（间隔>10断开）
    segments: List[Tuple[int, int]] = []
    cur_start = danger[0]
    for i in range(1, len(danger)):
        if danger[i] - danger[i - 1] > 10:
            segments.append((cur_start, danger[i - 1]))
            cur_start = danger[i]
    segments.append((cur_start, danger[-1]))

    return segments