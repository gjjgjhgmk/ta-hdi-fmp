import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(__file__)
PKG_ROOT = os.path.join(ROOT, '..')
sys.path.insert(0, os.path.abspath(PKG_ROOT))

from ta_hdi_fmp import (
    PlannerConfig,
    adaptive_hdi,
    compare_paths_approx,
    compute_discrete_curvature,
    get_informed_rrt_star_path_pool,
    score_path,
    verify_trajectory,
)


class TestFrameworkPy(unittest.TestCase):
    def setUp(self):
        self.cfg = PlannerConfig(
            demo_len=80,
            demo_dt=0.1,
            n_c=20,
            max_iter_cap=3000,
            time_budget_per_segment=0.2,
            via_cap_factor=2.0,
            rrt_shape_mean_tol=0.8,
            rrt_shape_max_tol=1.5,
        )

    def test_t2_curvature(self):
        path = np.array([[0, 1, 2, 3], [0, 1, 0, 1]], dtype=float)
        kappa = compute_discrete_curvature(path)
        self.assertEqual(kappa.shape[0], 2)
        self.assertGreater(kappa[0], 0.0)

    def test_t5_verify(self):
        obstacles = np.array([
            [50, 50, 3, 0, 0, 1],
            [20, 20, 6, 4, np.pi / 6, 2],
        ], dtype=float)

        collision_traj = np.array([[50, 50, 50], [50, 50, 50]], dtype=float)
        ok1, _ = verify_trajectory(collision_traj, obstacles, self.cfg)
        self.assertFalse(ok1)

        safe_traj = np.array([[5, 95], [5, 95]], dtype=float)
        ok2, _ = verify_trajectory(safe_traj, obstacles, self.cfg)
        self.assertTrue(ok2)

    def test_t6_score(self):
        demo_seg = np.array([[0, 5, 10], [0, 2, 5]], dtype=float)
        short = np.array([[0, 5, 10], [0, 2, 5]], dtype=float)
        long = np.array([[0, 2, 5, 7, 10], [0, 1, 3, 4, 5]], dtype=float)
        obstacles = np.array([[1000, 1000, 1, 0, 0, 1]], dtype=float)
        s_short = score_path(short, demo_seg, obstacles, self.cfg)
        s_long = score_path(long, demo_seg, obstacles, self.cfg)
        self.assertLess(s_short, s_long)

    def test_compare_paths(self):
        p1 = np.array([[0, 5, 10], [0, 5, 10]], dtype=float)
        p2 = np.array([[0, 5.2, 10], [0, 4.9, 10]], dtype=float)
        ok, details = compare_paths_approx(p1, p2, self.cfg)
        self.assertIn('mean_point_dist', details)
        self.assertIsInstance(ok, bool)

    def test_t4_hdi_density(self):
        rrt_path = np.array([[0, 5, 10, 15, 20], [0, 0, 0, 0, 0]], dtype=float)
        obstacles = np.array([[10, 0, 1.8, 0, 0, 1]], dtype=float)
        cfg_hdi = PlannerConfig(**self.cfg.__dict__)
        cfg_hdi.via_cap_factor = 10.0
        dense, _, is_danger = adaptive_hdi(rrt_path, obstacles, cfg_hdi, 1, 80, 0.1)
        self.assertGreater(dense.shape[1], 0)

        idx_d = np.where(is_danger)[0]
        idx_s = np.where(~is_danger)[0]
        self.assertGreaterEqual(idx_d.size, 3)
        self.assertGreaterEqual(idx_s.size, 3)

        d_spacing = np.mean(np.abs(np.diff(dense[0, idx_d])))
        s_spacing = np.mean(np.abs(np.diff(dense[0, idx_s])))
        self.assertLess(d_spacing, s_spacing)

    def test_t3_rrt_pool(self):
        np.random.seed(123)
        start = np.array([0.0, 0.0])
        goal = np.array([10.0, 10.0])
        obstacles = np.array([[5, 5, 1.5, 0, 0, 1]], dtype=float)

        cfg_reg = PlannerConfig(**self.cfg.__dict__)
        cfg_reg.time_budget_per_segment = 0.5
        cfg_reg.max_iter_cap = 5000

        pool = get_informed_rrt_star_path_pool(start, goal, obstacles, cfg_reg)
        self.assertGreater(len(pool), 0)
        self.assertGreater(pool[0].cost, 0.0)

        ref = pool[0].path
        ok_any = False
        for cand in pool:
            ok, _ = compare_paths_approx(ref, cand.path, cfg_reg)
            if ok:
                ok_any = True
                break
        self.assertTrue(ok_any)


if __name__ == '__main__':
    unittest.main(verbosity=2)
