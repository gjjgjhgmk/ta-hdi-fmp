"""demo_processing.py - 示教轨迹数据预处理

对应 MATLAB: func/demo_processing.m

输入: my_demos = [{'pos': np.array([x_line, y_line])}]
输出: Data 矩阵 (6 × N)
    Data[0]: 时间索引 j (1,2,3...demoLen, 重复 num_demos 次)
    Data[1]: 指数特征 exp(alpha * j / demoLen)
    Data[2]: x 坐标
    Data[3]: y 坐标
    Data[4]: vx (x方向速度)
    Data[5]: vy (y方向速度)
"""

import numpy as np
from typing import List, Dict, Tuple


def demo_processing(
    my_demos: List[Dict],
    demo_len: int = 200,
    demo_dt: float = 0.1,
    alpha: float = 0.1
) -> Tuple[np.ndarray, float]:
    """
    处理示教轨迹数据，生成 Data 矩阵用于模糊建模

    Args:
        my_demos: 示教轨迹列表，每个元素为 {'pos': np.array([2, demoLen])}
        demo_len: 每个示教轨迹的点数
        demo_dt: 采样时间步
        alpha: 指数特征系数

    Returns:
        Data: 6 × N 矩阵，N = demo_len * num_demos
        demo_dura: 示教时长，1/demo_dt
    """
    demo_num = len(my_demos)
    demo_dura = 1.0 / demo_dt
    demo_scale = 1.0

    # 数据增强：若 demo 数 < 6，创建副本（与 MATLAB 一致）
    if demo_num < 6:
        demo_up = [{'pos': d['pos'] + np.array([[0], [demo_scale]])} for d in my_demos]
        demo_down = [{'pos': d['pos'] - np.array([[0], [demo_scale]])} for d in my_demos]
        my_demos = my_demos + demo_up + demo_down

    total_num = 0
    data_list = []

    for demo in my_demos:
        pos = demo['pos']  # shape: (2, demo_len)

        for j in range(demo_len):
            total_num += 1

            # 时间索引 (1-based，与 MATLAB 一致)
            t = j + 1

            # 指数特征 exp(alpha * j / demo_len)
            exp_feat = np.exp(alpha * (j + 1) / demo_len)

            # 坐标
            x = pos[0, j]
            y = pos[1, j]

            # 速度计算：中心差分，首尾用前向/后向差分
            if j == 0:
                # 首点用前向差分
                vx = (pos[0, 1] - pos[0, 0]) / demo_dt
                vy = (pos[1, 1] - pos[1, 0]) / demo_dt
            elif j == demo_len - 1:
                # 尾点用后向差分
                vx = (pos[0, -1] - pos[0, -2]) / demo_dt
                vy = (pos[1, -1] - pos[1, -2]) / demo_dt
            else:
                # 中间点用中心差分
                vx = (pos[0, j + 1] - pos[0, j - 1]) / (2 * demo_dt)
                vy = (pos[1, j + 1] - pos[1, j - 1]) / (2 * demo_dt)

            data_list.append([t, exp_feat, x, y, vx, vy])

    Data = np.array(data_list).T  # shape: (6, total_num)
    return Data, demo_dura


def get_default_demo(demo_len: int = 200) -> Tuple[List[Dict], np.ndarray, np.ndarray]:
    """
    生成默认正弦示教轨迹，与 MATLAB run_single_trial.m 一致

    Returns:
        my_demos: 示教轨迹列表
        x_line: x 坐标数组
        y_line: y 坐标数组
    """
    x_line = np.linspace(0, 100, demo_len)
    y_line = 50 + 30 * np.sin(x_line * np.pi / 50)
    my_demos = [{'pos': np.array([x_line, y_line])}]
    return my_demos, x_line, y_line
