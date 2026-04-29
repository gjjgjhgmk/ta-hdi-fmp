"""push_repair.py - 穿障修复

对应 MATLAB: push_traj_clear.m

Post-process FMP 输出后修复障碍物穿透。
对于穿透障碍物的每个轨迹点，计算外向梯度并推动点远离。
推动后用高斯核平滑以避免引入新的拐点。
"""

import numpy as np
from typing import Tuple
from .geometry import signed_dist_and_grad


def push_traj_clear(
    traj: np.ndarray,
    obstacles: np.ndarray,
    cfg
) -> np.ndarray:
    """
    穿障修复

    Args:
        traj: 轨迹 (2, N)
        obstacles: 障碍物数组 (M, 6)
        cfg: PlannerConfig，需要包含:
            - push_target_margin: 推动后目标距离
            - push_max_iters: 最大迭代次数
            - push_smooth_sigma: 高斯平滑 sigma
            - pass_collision_margin: 碰撞检测裕量（优先使用）

    Returns:
        traj_out: 修复后的轨迹 (2, N)
    """
    traj_out = traj.copy()
    N = traj.shape[1]
    if N < 2:
        return traj_out

    # 确定推动目标距离（优先级：push_target_margin > safe_margin > pass_collision_margin）
    push_margin = cfg.safe_margin
    if hasattr(cfg, 'push_target_margin'):
        push_margin = cfg.push_target_margin  # 优先使用 push_target_margin
    elif hasattr(cfg, 'pass_collision_margin'):
        push_margin = cfg.pass_collision_margin

    max_iters = 3
    if hasattr(cfg, 'push_max_iters'):
        max_iters = cfg.push_max_iters

    smooth_sigma = 5.0
    if hasattr(cfg, 'push_smooth_sigma'):
        smooth_sigma = cfg.push_smooth_sigma

    for iteration in range(max_iters):
        any_fixed = False
        for i in range(N):
            pt = traj_out[:, i]
            d_min, grad_out = point_safe_push_gradient(pt, obstacles)
            if d_min < push_margin:
                # 按差值推动
                deficit = push_margin - d_min
                traj_out[:, i] = pt + grad_out * (deficit + 1e-4)
                any_fixed = True

        if not any_fixed:
            break

        # 重新平滑（保持端点固定）
        traj_out = smooth_traj(traj_out, smooth_sigma)

    return traj_out


def point_safe_push_gradient(
    pt: np.ndarray,
    obstacles: np.ndarray
) -> Tuple[float, np.ndarray]:
    """
    计算点的最小带符号距离和外向梯度

    Args:
        pt: 点坐标 (2,)
        obstacles: 障碍物数组 (M, 6)

    Returns:
        d_min: 最小带符号距离
        grad: 向外单位梯度 (2,)
    """
    d_min = float('inf')
    grad = np.array([0.0, 1.0])  # 默认向上

    for k in range(obstacles.shape[0]):
        d, g = signed_dist_and_grad(pt, obstacles[k, :])
        if d < d_min:
            d_min = d
            grad = g

    return d_min, grad


def smooth_traj(traj: np.ndarray, sigma: float) -> np.ndarray:
    """
    高斯平滑轨迹，保持端点固定

    Args:
        traj: 轨迹 (2, N)
        sigma: 高斯核 sigma

    Returns:
        平滑后的轨迹 (2, N)
    """
    N = traj.shape[1]
    if N < 3 or sigma < 1e-6:
        return traj

    hw = int(np.ceil(3 * sigma))
    x = np.arange(-hw, hw + 1, dtype=float)
    g = np.exp(-0.5 * (x / sigma) ** 2)
    g = g / np.sum(g)

    traj_out = traj.copy()
    for dim in range(2):
        row = traj[dim, :]
        row_padded = np.concatenate([
            np.repeat(row[0], hw),
            row,
            np.repeat(row[-1], hw)
        ])
        smoothed = np.convolve(row_padded, g, mode='valid')
        # 确保长度匹配
        if len(smoothed) >= N:
            smoothed = smoothed[:N]
        else:
            smoothed = np.interp(
                np.linspace(0, len(smoothed) - 1, N),
                np.arange(len(smoothed)),
                smoothed
            )
        traj_out[dim, :] = smoothed

    # 固定端点
    traj_out[:, 0] = traj[:, 0]
    traj_out[:, -1] = traj[:, -1]

    return traj_out
