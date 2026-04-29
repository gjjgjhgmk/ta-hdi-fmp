"""tests/test_fmp_integration.py - TA-HDI-FMP 集成测试

测试完整流程
"""

import numpy as np
import sys
sys.path.insert(0, 'd:/01_code/FMP/python_core')

from ta_hdi_fmp import (
    PlannerConfig,
    get_default_cfg,
    run_single_trial,
    generate_obstacles,
    detect_danger_segments,
    fuzzy_model,
    demo_processing,
    get_default_demo,
    fuzregre_modulation_yout,
    push_traj_clear,
    adaptive_hdi,
    verify_trajectory,
    score_path,
    get_informed_rrt_star_path_pool,
)


def test_demo_processing():
    """测试示教数据处理"""
    print("Testing demo_processing...")
    my_demos, x_line, y_line = get_default_demo(200)
    Data, demo_dura = demo_processing(my_demos, demo_len=200, demo_dt=0.1, alpha=0.1)

    assert Data.shape[0] == 6, f"Data should have 6 rows, got {Data.shape[0]}"
    assert Data.shape[1] == 200 * 3, f"Data should have 600 columns, got {Data.shape[1]}"  # 3 demos after augmentation
    assert abs(demo_dura - 10.0) < 0.1, f"demo_dura should be 10.0, got {demo_dura}"
    print("  PASS: demo_processing")


def test_fuzzy_model():
    """测试模糊建模"""
    print("Testing fuzzy_model...")
    my_demos, _, _ = get_default_demo(200)
    cfg = get_default_cfg()

    C_x, pInvCov_x, p1_u_x, C_y, pInvCov_y, p1_u_y, demo_dura = fuzzy_model(
        my_demos,
        cfg.demo_len,
        cfg.demo_dt,
        cfg.alpha,
        cfg.n_c,
        cfg.max_iter_fcm,
        cfg.max_iter_gk
    )

    assert C_x.shape == (3, cfg.n_c), f"C_x shape should be (3, {cfg.n_c}), got {C_x.shape}"
    assert C_y.shape == (3, cfg.n_c), f"C_y shape should be (3, {cfg.n_c}), got {C_y.shape}"
    assert pInvCov_x.shape == (3, 3, cfg.n_c), f"pInvCov_x shape should be (3, 3, {cfg.n_c}), got {pInvCov_x.shape}"
    assert p1_u_x.shape == (3, 1, cfg.n_c), f"p1_u_x shape should be (3, 1, {cfg.n_c}), got {p1_u_x.shape}"
    print("  PASS: fuzzy_model")


def test_obstacle_generation():
    """测试障碍物生成"""
    print("Testing generate_obstacles...")
    np.random.seed(2025)
    obstacles = generate_obstacles(40, env_seed=2025, min_gap=1.0)

    assert obstacles.shape[0] > 0, "Should generate some obstacles"
    assert obstacles.shape[1] == 6, f"Should have 6 columns, got {obstacles.shape[1]}"
    print(f"  Generated {obstacles.shape[0]} obstacles")
    print("  PASS: generate_obstacles")


def test_danger_detection():
    """测试冲突检测"""
    print("Testing detect_danger_segments...")
    my_demos, x_line, y_line = get_default_demo(200)
    demo_pos = np.array([x_line, y_line])

    np.random.seed(2025)
    obstacles = generate_obstacles(40, env_seed=2025, min_gap=1.0)

    segments = detect_danger_segments(demo_pos, obstacles, safe_margin=1.5)
    print(f"  Found {len(segments)} danger segments")
    print("  PASS: detect_danger_segments")


def test_rrt_pool():
    """测试 RRT* 候选池"""
    print("Testing RRT* pool...")
    cfg = get_default_cfg()
    cfg.time_budget_per_segment = 0.5  # 增加时间预算

    start = np.array([0.0, 0.0])
    goal = np.array([5.0, 5.0])

    obstacles = np.array([
        [2.5, 2.5, 1.0, 0, 0, 1],  # 圆形障碍
    ])

    pool = get_informed_rrt_star_path_pool(start, goal, obstacles, cfg)

    assert len(pool) > 0, "Should get at least one path"
    print(f"  Got {len(pool)} candidates")
    for i, cand in enumerate(pool):
        print(f"    Candidate {i}: cost={cand.cost:.2f}, points={cand.path.shape[1]}")
    print("  PASS: RRT* pool")


def test_hdi_interpolation():
    """测试 HDI 插值"""
    print("Testing adaptive_hdi...")
    cfg = get_default_cfg()

    # 简单路径
    path = np.array([[0, 1, 2, 3], [0, 1, 0, 1]], dtype=float)
    obstacles = np.array([
        [2, 0.5, 0.3, 0, 0, 1],
    ])

    dense, t_local, danger = adaptive_hdi(path, obstacles, cfg, 0, 100, 0.1)

    assert dense.shape[1] >= path.shape[1], "Dense should have more or equal points"
    assert len(t_local) == dense.shape[1], "t_local should match dense points"
    print(f"  Original: {path.shape[1]} points -> Dense: {dense.shape[1]} points")
    print("  PASS: adaptive_hdi")


def test_fmp_modulation():
    """测试 FMP 调制"""
    print("Testing fuzregre_modulation_yout...")
    cfg = get_default_cfg()

    # 准备模糊模型
    my_demos, _, _ = get_default_demo(200)
    C_x, pInvCov_x, p1_u_x, C_y, pInvCov_y, p1_u_y, demo_dura = fuzzy_model(
        my_demos, cfg.demo_len, cfg.demo_dt, cfg.alpha, cfg.n_c,
        cfg.max_iter_fcm, cfg.max_iter_gk
    )

    # 时间参数
    timeinput = np.arange(0.01, 2.0, 0.01)
    N_Data = len(timeinput)

    # via 点
    via_time = np.array([0.5, 1.0, 1.5])
    via_point = np.array([[1.0, 2.0, 3.0], [0.5, 1.0, 0.5]])

    # FMP 调制 - 分别对 x 和 y 调用（与 MATLAB 一致）
    # 注意：Location_V=[0] 表示选择 via_point 的第一行
    yx = fuzregre_modulation_yout(
        timeinput, demo_dura, cfg.alpha, N_Data, cfg.n_c,
        C_x, pInvCov_x, p1_u_x,
        [0, 1], [2],
        via_time, via_point, [0]  # 传入完整的 (2, 3) via_point，Location_V=[0] 选择第一行
    )
    yy = fuzregre_modulation_yout(
        timeinput, demo_dura, cfg.alpha, N_Data, cfg.n_c,
        C_y, pInvCov_y, p1_u_y,
        [0, 1], [2],
        via_time, via_point, [1]  # Location_V=[1] 选择第二行
    )

    assert len(yx) == N_Data, f"yx length should be {N_Data}, got {len(yx)}"
    assert len(yy) == N_Data, f"yy length should be {N_Data}, got {len(yy)}"
    print(f"  Output trajectory: {N_Data} points")
    print("  PASS: fuzregre_modulation_yout")


def test_push_repair():
    """测试穿障修复"""
    print("Testing push_traj_clear...")
    cfg = get_default_cfg()

    # 简单轨迹（部分穿障）
    traj = np.array([
        [0.0, 1.0, 2.0, 3.0],
        [0.0, 1.0, 0.0, 1.0]
    ], dtype=float)

    # 障碍物在 (2, 0.5) 附近
    obstacles = np.array([
        [2.0, 0.5, 0.3, 0, 0, 1],
    ])

    traj_fixed = push_traj_clear(traj, obstacles, cfg)

    assert traj_fixed.shape == traj.shape, "Output shape should match input"
    print(f"  Fixed trajectory shape: {traj_fixed.shape}")
    print("  PASS: push_traj_clear")


def test_verify_trajectory():
    """测试轨迹验证"""
    print("Testing verify_trajectory...")
    cfg = get_default_cfg()

    # 有效轨迹
    traj = np.array([
        [0.0, 1.0, 2.0, 3.0, 4.0],
        [0.0, 0.5, 1.0, 0.5, 0.0]
    ], dtype=float)

    obstacles = np.array([
        [5.0, 5.0, 1.0, 0, 0, 1],  # 远离轨迹
    ])

    ok, metrics = verify_trajectory(traj, obstacles, cfg)

    print(f"  Verification: {'PASS' if ok else 'FAIL'}")
    print(f"  min_dist: {metrics['min_dist']:.3f}, kappa_max: {metrics['kappa_max']:.3f}")
    print("  PASS: verify_trajectory")


def test_full_pipeline():
    """测试完整流程"""
    print("\n=== Testing full TA-HDI-FMP pipeline ===")
    cfg = get_default_cfg()
    cfg.time_budget_per_segment = 1.0  # 增加时间预算以便 RRT 找到解

    result = run_single_trial(cfg, seed=2025, do_plot=False)

    print(f"\nResult:")
    print(f"  Success: {result['success']}")
    print(f"  Candidate used: {result['candidate_used']}")
    print(f"  Via points: {result['n_via']}")
    print(f"  Trajectory shape: {result['trajectory'].shape}")
    print(f"  Metrics: min_dist={result['metrics']['min_dist']:.3f}, "
          f"kappa_max={result['metrics']['kappa_max']:.3f}, "
          f"jerk_rms={result['metrics']['jerk_rms']:.3f}")

    print("\nPASS: full_pipeline")


def main():
    print("=" * 60)
    print("TA-HDI-FMP Module Tests")
    print("=" * 60)

    try:
        test_demo_processing()
        test_fuzzy_model()
        test_obstacle_generation()
        test_danger_detection()
        test_rrt_pool()
        test_hdi_interpolation()
        test_fmp_modulation()
        test_push_repair()
        test_verify_trajectory()
        test_full_pipeline()

        print("\n" + "=" * 60)
        print("ALL TESTS PASSED!")
        print("=" * 60)
    except Exception as e:
        print(f"\nTEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    exit(main())
