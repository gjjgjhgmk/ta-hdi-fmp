"""fuzzy_clustering.py - FCM + GK 模糊聚类

对应 MATLAB: func/fuzzy_clustering.m

输出:
    C: 聚类中心 (D_P × N_C)
    AFPCD: 平均模糊伪 Cauchy 距离
    nIn: 归一化逆协方差矩阵 (D_P × D_P × N_C)
    cov: 协方差矩阵 (D_P × D_P × N_C) - 修复了 MATLAB bug
    U: 隶属度矩阵 (N_C × N_P)
"""

import numpy as np
from typing import Tuple


def ensure_psd(matrix: np.ndarray, epsilon: float = 1e-6) -> np.ndarray:
    """确保矩阵是数值稳定的（添加小量到对角线）"""
    eigenvalues = np.linalg.eigvalsh(matrix)
    if eigenvalues.min() <= epsilon:
        matrix = matrix + np.eye(matrix.shape[0]) * (epsilon - eigenvalues.min() + epsilon)
    return matrix


def fuzzy_clustering(
    P: np.ndarray,
    len_data: int,
    N_C: int,
    maxIter_fcm: int = 30,
    maxIter_gk: int = 30
) -> Tuple[np.ndarray, float, np.ndarray, np.ndarray, np.ndarray]:
    """
    FCM + GK 模糊聚类

    Args:
        P: 数据矩阵 (D_P × N_P)
        len_data: 使用的数据长度
        N_C: 聚类数
        maxIter_fcm: FCM 最大迭代
        maxIter_gk: GK 最大迭代

    Returns:
        C: 聚类中心
        AFPCD: 平均模糊伪 Cauchy 距离
        nIn: 归一化逆协方差矩阵
        cov: 协方差矩阵 (修复 MATLAB bug，使用完整矩阵)
        U: 隶属度矩阵
    """
    D_P, N_P = P.shape
    ones_DP_x1 = np.ones((D_P, 1))

    # 取前 len_data 列数据
    Ps = P[:, :len_data]

    # 初始化聚类中心（等间距采样，与 MATLAB 一致）
    sec_2 = int(0.5 * len_data / (N_C + 1))
    sec = int((len_data - 2 * sec_2) / (N_C - 1))
    C = Ps[:, sec_2::sec]
    C = C[:, :N_C]  # shape: (D_P, N_C)

    ones_NC_x1 = np.ones((N_C, 1))

    # 计算距离平方的辅助函数
    def dist2_func(a, b):
        # a: (D_P, N_P), b: (D_P, N_C)
        # 返回: (N_C, N_P)，每列是一个聚类中心到所有点的距离
        diff = a[:, :, np.newaxis] - b[:, np.newaxis, :]
        return np.sum(diff ** 2, axis=0).T

    dist2 = dist2_func(Ps, C)
    idxs = (dist2 == 0)
    dist2[idxs] = 1e-6  # 避免除零

    # FCM 迭代
    tmp = 1.0 / dist2
    U = tmp / np.sum(tmp, axis=0, keepdims=True)
    U2 = U ** 2

    for kIter in range(maxIter_fcm):
        rowSumU2 = np.sum(U2, axis=1, keepdims=True)
        C = (Ps @ U2.T) / (ones_DP_x1 @ rowSumU2.T)
        dist2 = dist2_func(Ps, C)
        tmp = 1.0 / dist2
        U = tmp / np.sum(tmp, axis=0, keepdims=True)
        U2 = U ** 2

    # GK 算法（计算协方差矩阵）
    nIn = np.zeros((D_P, D_P, N_C))
    cov = np.zeros((D_P, D_P, N_C))

    if maxIter_gk <= 0:
        # 无 GK，使用单位矩阵
        eyes = np.eye(D_P)
        for i in range(N_C):
            nIn[:, :, i] = eyes
        AFPCD = np.sum(1.0 / np.sum(1.0 / dist2, axis=0)) / N_P
    else:
        a = 1.0 / D_P

        if maxIter_gk == 1:
            # 单次 GK
            rowSumU2 = np.sum(U2, axis=1, keepdims=True)
            MD2 = np.zeros((N_C, len_data))

            for i in range(N_C):
                dist = Ps - C[:, i:i+1] @ np.ones((1, len_data))
                U2D = (ones_DP_x1 @ U2[i:i+1, :]) * dist
                cov_i = (dist @ U2D.T) / rowSumU2[i, 0]
                cov_i = ensure_psd(cov_i)

                det_cov = np.linalg.det(cov_i)
                if det_cov <= 0:
                    det_cov = 1e-10

                try:
                    inv_cov = np.linalg.inv(cov_i)
                except np.linalg.LinAlgError:
                    inv_cov = np.linalg.pinv(cov_i)

                nIn_i = (det_cov ** a) * inv_cov
                MD2[i, :] = np.sum(dist * (nIn_i @ dist), axis=0)
                nIn[:, :, i] = nIn_i
                # 修复 MATLAB bug：直接存储完整协方差矩阵（不用 cov_i^0.5）
                cov[:, :, i] = cov_i

            AFPCD = np.sum(1.0 / np.sum(1.0 / MD2, axis=0)) / N_P
        else:
            # 多次 GK 迭代
            MD2 = dist2.copy()
            for kIter in range(maxIter_gk - 1):
                rowSumU2 = np.sum(U2, axis=1, keepdims=True)
                C = (Ps @ U2.T) / (ones_DP_x1 @ rowSumU2.T)
                for i in range(N_C):
                    dist = Ps - C[:, i:i+1] @ np.ones((1, len_data))
                    U2D = (ones_DP_x1 @ U2[i:i+1, :]) * dist
                    cov_i = (dist @ U2D.T) / rowSumU2[i, 0]
                    cov_i = ensure_psd(cov_i)
                    det_cov = np.linalg.det(cov_i)
                    if det_cov <= 0:
                        det_cov = 1e-10

                    try:
                        inv_cov = np.linalg.inv(cov_i)
                    except np.linalg.LinAlgError:
                        inv_cov = np.linalg.pinv(cov_i)

                    nIn_i = (det_cov ** a) * inv_cov
                    MD2[i, :] = np.sum(dist * (nIn_i @ dist), axis=0)

                tmp = 1.0 / MD2
                U = tmp / np.sum(tmp, axis=0, keepdims=True)
                U2 = U ** 2

            # 最后一次 GK 迭代
            rowSumU2 = np.sum(U2, axis=1, keepdims=True)
            C = (Ps @ U2.T) / (ones_DP_x1 @ rowSumU2.T)
            MD2 = np.zeros((N_C, len_data))

            for i in range(N_C):
                dist = Ps - C[:, i:i+1] @ np.ones((1, len_data))
                U2D = (ones_DP_x1 @ U2[i:i+1, :]) * dist
                cov_i = (dist @ U2D.T) / rowSumU2[i, 0]
                cov_i = ensure_psd(cov_i)
                det_cov = np.linalg.det(cov_i)
                if det_cov <= 0:
                    det_cov = 1e-10

                try:
                    inv_cov = np.linalg.inv(cov_i)
                except np.linalg.LinAlgError:
                    inv_cov = np.linalg.pinv(cov_i)

                nIn_i = (det_cov ** a) * inv_cov
                MD2[i, :] = np.sum(dist * (nIn_i @ dist), axis=0)
                nIn[:, :, i] = nIn_i
                # 修复 MATLAB bug：直接存储完整协方差矩阵
                cov[:, :, i] = cov_i

            AFPCD = np.sum(1.0 / np.sum(1.0 / MD2, axis=0)) / N_P

    return C, AFPCD, nIn, cov, U
