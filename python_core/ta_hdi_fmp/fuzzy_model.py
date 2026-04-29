"""fuzzy_model.py - 模糊建模组合模块

对应 MATLAB: func/fuzzymodellingCandGK.m
组合 fuzzy_clustering + fuzregre_param_train_t1

输入: Data 矩阵 (6 × N)
输出:
    C_x, pInvCov_x, p1_u_x: x 坐标的模糊模型参数
    C_y, pInvCov_y, p1_u_y: y 坐标的模糊模型参数
"""

import numpy as np
from typing import Tuple, List, Dict
from .fuzzy_clustering import fuzzy_clustering
from .demo_processing import demo_processing


def fuzzy_model(
    my_demos: List[Dict],
    demo_len: int = 200,
    demo_dt: float = 0.1,
    alpha: float = 0.1,
    N_C: int = 40,
    maxIter_fcm: int = 30,
    maxIter_gk: int = 30
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """
    模糊建模 - 生成 x 和 y 坐标的模型参数

    Args:
        my_demos: 示教轨迹列表
        demo_len: 示教轨迹点数
        demo_dt: 采样时间步
        alpha: 指数特征系数
        N_C: 聚类数
        maxIter_fcm: FCM 最大迭代
        maxIter_gk: GK 最大迭代

    Returns:
        C_x, pInvCov_x, p1_u_x: x 坐标模型参数
        C_y, pInvCov_y, p1_u_y: y 坐标模型参数
        demo_dura: 示教时长
    """
    # 数据预处理
    Data, demo_dura = demo_processing(my_demos, demo_len, demo_dt, alpha)
    N_P = Data.shape[1]  # 完整数据点数（600）

    # x 坐标建模: 使用 Data[0:3, :] = [t, exp_feat, x]
    # MATLAB 索引是 1-based，所以 Location_X = [1 2] -> Python [0, 1]
    Data_x = Data[[0, 1, 2], :]
    C_x, _, pInvCov_x, _, U_x = fuzzy_clustering(Data_x, N_P, N_C, maxIter_fcm, maxIter_gk)
    p1_u_x = fuzregre_param_train(Data_x, U_x, N_C, [0, 1], [2])

    # y 坐标建模: 使用 Data[[0,1,3], :] = [t, exp_feat, y]
    Data_y = Data[[0, 1, 3], :]
    C_y, _, pInvCov_y, _, U_y = fuzzy_clustering(Data_y, N_P, N_C, maxIter_fcm, maxIter_gk)
    p1_u_y = fuzregre_param_train(Data_y, U_y, N_C, [0, 1], [2])

    return C_x, pInvCov_x, p1_u_x, C_y, pInvCov_y, p1_u_y, demo_dura


def fuzregre_param_train(
    Data_train: np.ndarray,
    U: np.ndarray,
    N_C: int,
    Location_X: List[int],
    Location_Y: List[int]
) -> np.ndarray:
    """
    模糊回归参数训练

    对应 MATLAB: func/fuzregre_param_train_t1.m

    MATLAB 代码:
        Wi = U(i,idx);  -- row vector (1, N_i) in MATLAB
        Wii = Wi'*ones(1,1+slx);  -- (N_i, 1) * (1, 1+slx) -> (N_i, 1+slx)
        X1 = [ones(size(zc,2),1), zc(Location_X,:)'];  -- (N_i, 1+slx)
        p1_u = (X1'*(Wii.*X1))^(-1) * X1' * (Wi'.*Y1);

    Args:
        Data_train: 训练数据 (D_P × N_P)
        U: 隶属度矩阵 (N_C × N_P)
        N_C: 聚类数
        Location_X: X 特征索引 (0-based Python)
        Location_Y: Y 特征索引 (0-based Python)

    Returns:
        p1_u: 回归参数 (1+slx, sly, N_C)
    """
    # 找到每个数据点属于哪个聚类
    jl = np.argmax(U, axis=0)  # shape: (N_P,)

    slx = len(Location_X)
    sly = len(Location_Y)

    # p1_u shape: (1+slx, sly, N_C)
    p1_u = np.zeros((1 + slx, sly, N_C))

    for i in range(N_C):
        # 找到属于第 i 类的数据点
        idx = (jl == i)
        zc = Data_train[:, idx]  # shape: (D_P, N_i)
        N_i = zc.shape[1]

        if N_i == 0:
            continue

        # Wi = U(i,idx) - 在 Python 中是 (N_i,) 数组
        Wi = U[i, idx]  # shape: (N_i,)

        # Wii = Wi' * ones(1, 1+slx) -> (N_i, 1+slx)
        # 每个元素 Wii[j,k] = Wi[j]
        Wii = Wi[:, np.newaxis] * np.ones((1, 1 + slx))  # (N_i, 1+slx)

        # X1 = [ones(N_i,1), zc(Location_X,:)] -> (N_i, 1+slx)
        X1 = np.hstack([np.ones((N_i, 1)), zc[Location_X, :].T])  # (N_i, 1+slx)

        # Y1 = zc(Location_Y,:)' -> (N_i, sly)
        Y1 = zc[Location_Y, :].T  # (N_i, sly)

        # Wi' in MATLAB is (N_i, 1) after transpose, but in Python Wi is already (N_i,)
        # Wi' .* Y1 -> (N_i, 1) .* (N_i, sly) -> (N_i, sly)
        Wi_col = Wi[:, np.newaxis]  # (N_i, 1)
        WiY1 = Wi_col * Y1  # (N_i, sly)

        # Wii .* X1 -> (N_i, 1+slx) .* (N_i, 1+slx) -> (N_i, 1+slx)
        Wii_X1 = Wii * X1

        # X1' * (Wii .* X1) -> (1+slx, N_i) * (N_i, 1+slx) -> (1+slx, 1+slx)
        X1T = X1.T  # (1+slx, N_i)
        X1T_Wii_X1 = X1T @ Wii_X1  # (1+slx, 1+slx)

        # 确保矩阵可逆
        cond = np.linalg.cond(X1T_Wii_X1)
        if cond > 1e10:
            X1T_Wii_X1 += np.eye(1 + slx) * 1e-6

        # X1' * (Wi' .* Y1) -> (1+slx, N_i) * (N_i, sly) -> (1+slx, sly)
        X1T_WiY1 = X1T @ WiY1  # (1+slx, sly)

        p1_u[:, :, i] = np.linalg.inv(X1T_Wii_X1) @ X1T_WiY1

    return p1_u