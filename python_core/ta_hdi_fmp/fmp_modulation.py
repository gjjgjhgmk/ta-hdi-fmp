"""fmp_modulation.py - FMP 模糊回归调制

对应 MATLAB: func/fuzregre_modulation_yout.m

这是最核心的移植模块，实现：
1. 模糊隶属度计算（基于马氏距离）
2. 加权回归输出
3. via 点约束注入（switch + add 机制）
"""

import numpy as np
from typing import Tuple, List


def fuzregre_modulation_yout(
    timeinput: np.ndarray,
    demo_dura: float,
    alpha: float,
    N_Data: int,
    N_C: int,
    C1: np.ndarray,
    pInvCov1: np.ndarray,
    p1_u1: np.ndarray,
    Location_X: List[int],
    Location_Y: List[int],
    via_time: np.ndarray,
    via_point: np.ndarray,
    Location_V: List[int]
) -> Tuple[np.ndarray, np.ndarray]:
    """
    FMP 模糊回归调制

    Args:
        timeinput: 输出时间点序列 (N_Data,)
        demo_dura: 示教时长，1/demo_dt
        alpha: 指数特征系数
        N_Data: 输出点数
        N_C: 聚类数
        C1: 聚类中心 (D_C, N_C)
        pInvCov1: 逆协方差矩阵 (D_C, D_C, N_C)
        p1_u1: 回归参数 (1+len_X, len_Y, N_C)
        Location_X: X 特征索引 (0-based Python)
        Location_Y: Y 特征索引 (0-based Python)
        via_time: via 点时间序列
        via_point: via 点坐标 (2, N_via)
        Location_V: via 点使用的特征索引 (0-based Python)

    Returns:
        yx, yy: 调制后的轨迹 (N_Data,)
    """
    D_y = len(Location_Y)
    D_C = C1.shape[0]
    N_via = len(via_time)

    # Data_test 构造（与 MATLAB 一致）
    # Data_test = [demo_dura*timeinput; exp(alpha*timeinput/timeinput(end))]
    Data_test = np.vstack([
        demo_dura * timeinput,
        np.exp(alpha * timeinput / timeinput[-1])
    ])  # (2, N_Data)

    # ===== 计算 MD2（via 点马氏距离）=====
    MD2 = np.zeros((N_C, N_via))
    for i in range(N_C):
        # dist = [via_time; exp(alpha*via_time/(Data_test(1,end))); via_point(Location_V, :)] - C1(:, i)
        dist = np.vstack([
            via_time,
            np.exp(alpha * via_time / Data_test[0, -1]),
            via_point[Location_V, :]
        ]) - C1[:, i:i+1]  # (D_C, N_via)

        pInvCov_i = pInvCov1[:, :, i]
        MD2[i, :] = np.sum(dist * (pInvCov_i @ dist), axis=0)

    # 隶属度
    temp = 1.0 / MD2
    U_via = temp / np.sum(temp, axis=0, keepdims=True)  # (N_C, N_via)
    jl = np.argmax(U_via, axis=0)  # 每个 via 点属于哪个聚类

    # ===== via 点分类：switch 和 add =====
    if N_via == 0:
        # 没有 via 点，直接使用原始模型输出
        via_time1 = np.array([])
        via_point1 = np.array([]).reshape(2, 0)
        via_time2 = np.array([])
        via_point2 = np.array([]).reshape(2, 0)
        U_via2 = np.array([]).reshape(N_C, 0)
        N_via1 = 0
        N_via2 = 0
        jl1 = []
        jl2 = []
    else:
        via_time1 = [via_time[0]]
        via_time2_list = []
        via_point1 = via_point[:, 0:1]
        via_point2_list = []
        N_via1 = 1
        N_via2 = 0
        U_via2_list = []
        jl1 = [jl[0]]
        jl2 = []

        for i in range(1, N_via):
            if jl[i] in jl[:i]:
                # add: 已有同类
                via_time2_list.append(via_time[i])
                via_point2_list.append(via_point[:, i])
                N_via2 += 1
                U_via2_list.append(U_via[:, i])
                jl2.append(jl[i])
            else:
                # switch: 新类
                via_time1.append(via_time[i])
                via_point1 = np.hstack([via_point1, via_point[:, i:i+1]])
                N_via1 += 1
                jl1.append(jl[i])

        via_time1 = np.array(via_time1)
        via_point1 = np.array(via_point1)  # 已经是 (2, N_via1) 形状
        if via_time2_list:
            via_time2 = np.array(via_time2_list)
            via_point2 = np.array(via_point2_list).T  # (N_via2, 2) -> (2, N_via2)
            U_via2 = np.array(U_via2_list).T
        else:
            via_time2 = np.array([])
            via_point2 = np.array([]).reshape(2, 0)
            U_via2 = np.array([]).reshape(N_C, 0)

    # ===== 修改聚类中心 C（switch 机制）=====
    C = C1.copy()
    p1_u = p1_u1.copy()

    for i in range(N_via1):
        # 找到 Data_test 中最接近 via_time1[i] 的时间点
        idx = np.argmin(np.abs(Data_test[0, :] - via_time1[i]))
        C_index = jl1[i]
        # C([1:end-1], C_index) = Data_test(:, idx) + 1e-7
        C[:-1, C_index] = Data_test[:, idx] + 1e-7
        # p1_u(1,:,C_index) = (via_point1(Location_V,i) - (datax1'*p1_u_delete)')'
        datax1 = Data_test[Location_X, idx]  # shape: (len_X,)
        p1_u_delete = p1_u[1:, :, C_index]  # shape: (len_X, sly)

        # MATLAB: datax1' is (1, len_X), p1_u_delete is (len_X, sly), result is (1, sly)
        # Python: datax1 is (len_X,), p1_u_delete is (len_X, sly)
        # datax1[:, np.newaxis].T is (1, len_X), same as datax1' in MATLAB
        datax1_row = datax1[:, np.newaxis].T  # (1, len_X)
        p1_u_row = (datax1_row @ p1_u_delete)  # (1, sly)
        p1_u[0, :, C_index] = via_point1[Location_V[0], i] - p1_u_row

    # ===== 添加新聚类中心（add 机制）=====
    d_pI1, d_pI2 = pInvCov1.shape[0], pInvCov1.shape[1]
    d_pu1, d_pu2 = p1_u1.shape[0], p1_u1.shape[1]

    N_C1 = N_C
    if N_via2 > 0:
        # 扩展 C, pInvCov, p1_u
        C2 = np.hstack([C, np.zeros((D_C, N_via2))])
        pInvCov2 = np.zeros((d_pI1, d_pI2, N_C + N_via2))
        pInvCov2[:, :, :N_C] = pInvCov1
        p1_u2 = np.zeros((d_pu1, d_pu2, N_C + N_via2))
        p1_u2[:, :, :N_C] = p1_u1

        for i in range(N_via2):
            idx = np.argmin(np.abs(Data_test[0, :] - via_time2[i]))
            C2[:, N_C + i] = C1[:, jl2[i]]
            C2[:-1, N_C + i] = Data_test[:, idx] + 1e-7

            # 逆协方差加权平均
            U_via2_current = U_via2[:, i:i+1].reshape(1, 1, N_C)
            pInvCov_temp = pInvCov1 * np.broadcast_to(U_via2_current, pInvCov1.shape)
            pInvCov2[:, :, N_C + i] = np.sum(pInvCov_temp, axis=2)

            # 回归参数
            p1_u2[:, :, N_C + i] = p1_u1[:, :, jl2[i]]
            p1_u_delete = p1_u2[1:, :, N_C + i]  # shape: (len_X, sly)

            # MATLAB: Data_test(Location_X, idx) is (len_X,), p1_u_delete is (len_X, sly)
            # Data_test(Location_X, idx)' * p1_u_delete is (1, sly)
            datax1_row = Data_test[Location_X, idx][:, np.newaxis].T  # (1, len_X)
            p1_u_row = (datax1_row @ p1_u_delete)  # (1, sly)
            p1_u2[0, :, N_C + i] = via_point2[Location_V[0], i] - p1_u_row

        N_C1 = N_C + N_via2
    else:
        C2 = C
        pInvCov2 = pInvCov1
        p1_u2 = p1_u
        N_C1 = N_C

    # ===== 最终输出计算 =====
    MD2_final = np.zeros((N_C1, N_Data))
    for i in range(N_C1):
        dist = Data_test[Location_X, :] - C2[Location_X, i:i+1]
        pInvCov_i = pInvCov2[np.ix_(*[Location_X]*2, [i])].squeeze()
        MD2_final[i, :] = np.sum(dist * (pInvCov_i @ dist), axis=0)

    temp = 1.0 / MD2_final
    U = temp / np.sum(temp, axis=0, keepdims=True)  # (N_C1, N_Data)

    # 加权回归输出（支持 x 和 y 分离输出）
    y_x = np.zeros(N_Data)
    y_y = np.zeros(N_Data)
    for j in range(N_C1):
        X1 = np.vstack([np.ones(N_Data), Data_test[Location_X, :]])  # (1+len_X, N_Data)
        # p1_u2[:, :, j] shape: (1+slx, sly) where sly = len(Location_Y)
        p1_u_j = p1_u2[:, :, j]  # (1+slx, sly)
        y_j = (X1.T @ p1_u_j).T  # (sly, N_Data)
        # U[j, :] shape: (N_Data,), 广播计算
        y_x = y_x + y_j[0, :] * U[j, :]
        if D_y > 1:
            y_y = y_y + y_j[1, :] * U[j, :]

    # 返回：yx 必返回，yy 仅当 D_y > 1 时有效
    return y_x, y_y
