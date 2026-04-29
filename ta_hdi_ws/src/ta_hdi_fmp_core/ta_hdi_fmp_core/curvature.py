from __future__ import annotations

import numpy as np


def compute_discrete_curvature(path: np.ndarray) -> np.ndarray:
    # path: 2xN
    p = np.asarray(path, dtype=float)
    n = p.shape[1]
    if n < 3:
        return np.zeros((0,), dtype=float)

    kappa = np.zeros((n - 2,), dtype=float)
    for i in range(1, n - 1):
        v1 = p[:, i] - p[:, i - 1]
        v2 = p[:, i + 1] - p[:, i]
        den = np.linalg.norm(v1) * np.linalg.norm(v2) * np.linalg.norm(v1 + v2)
        if den < 1e-10:
            kappa[i - 1] = 0.0
        else:
            cross_val = v1[0] * v2[1] - v1[1] * v2[0]
            kappa[i - 1] = 2.0 * abs(cross_val) / den
    return kappa
