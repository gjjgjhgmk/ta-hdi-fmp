import numpy as np

from ta_hdi_fmp.curvature import compute_discrete_curvature
from ta_hdi_fmp.geometry import min_point_to_obstacles, point_to_obstacle_distance


def test_curvature_basic():
    path = np.array([[0, 1, 2, 3], [0, 1, 0, 1]], dtype=float)
    kappa = compute_discrete_curvature(path)
    assert kappa.shape[0] == 2
    assert kappa[0] > 0


def test_geometry_circle_and_obb():
    circle = np.array([0, 0, 2, 0, 0, 1], dtype=float)
    obb = np.array([5, 0, 4, 2, 0, 2], dtype=float)
    d1 = point_to_obstacle_distance([3, 0], circle)
    d2 = point_to_obstacle_distance([5, 0], obb)
    assert d1 > 0
    assert d2 < 0.1

    obs = np.vstack([circle, obb])
    dmin = min_point_to_obstacles([5, 0], obs)
    assert dmin <= d2 + 1e-9
