% =========================================================================
% TA-HDI-FMP 框架层测试脚本（T2~T7）
% 运行方式：
%   cd('d:/01_Code/FMP/FMP');
%   test_framework
% =========================================================================
clc;
fprintf('\n===== TA-HDI-FMP 框架测试开始 =====\n');

addpath(pwd);
addpath(fullfile(pwd, 'func'));

pass_count = 0;
fail_count = 0;

%% T2: compute_discrete_curvature
try
    path = [0, 1, 2, 3; 0, 1, 0, 1];
    kappa = compute_discrete_curvature(path);
    assert(kappa(1) > 0, 'T2 失败：首个曲率应大于 0');
    assert(numel(kappa) == 2, 'T2 失败：N=4 时内点曲率数量应为 2');
    fprintf('[PASS] T2 曲率函数基础断言通过\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('[FAIL] T2 曲率函数失败: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% 准备公共测试配置
cfg = make_test_cfg();

%% T5: verify_trajectory
try
    obstacles = [50, 50, 3, 0, 0, 1; 20, 20, 6, 4, pi/6, 2];

    collision_traj = [50, 50, 50; 50, 50, 50];
    [pass1, ~] = verify_trajectory(collision_traj, obstacles, cfg);
    assert(~pass1, 'T5 失败：碰撞轨迹应为 fail');

    safe_traj = [5, 95; 5, 95];
    [pass2, ~] = verify_trajectory(safe_traj, obstacles, cfg);
    assert(pass2, 'T5 失败：安全轨迹应为 pass');

    fprintf('[PASS] T5 轨迹验收函数断言通过\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('[FAIL] T5 轨迹验收函数失败: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% T6: score_path
try
    demo_seg = [0, 5, 10; 0, 2, 5];
    short_path = [0, 5, 10; 0, 2, 5];
    long_path = [0, 2, 5, 7, 10; 0, 1, 3, 4, 5];
    obstacles = [1000, 1000, 1, 0, 0, 1]; % 远离路径，避免风险项干扰过强

    s_short = score_path(short_path, demo_seg, obstacles, cfg);
    s_long = score_path(long_path, demo_seg, obstacles, cfg);
    assert(s_short < s_long, 'T6 失败：短路径分数应小于长路径');

    fprintf('[PASS] T6 候选路径打分断言通过\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('[FAIL] T6 候选路径打分失败: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% T3: RRT 候选池 + 近似回归
try
    rng(123);
    start_pos = [0, 0];
    goal_pos = [10, 10];
    obstacles = [5, 5, 1.5, 0, 0, 1];

    pool = get_Informed_RRT_Star_path_pool(start_pos, goal_pos, obstacles, cfg);
    assert(~isempty(pool), 'T3 失败：候选池为空');
    assert(isstruct(pool), 'T3 失败：候选池应为结构体数组');
    assert(pool(1).cost > 0, 'T3 失败：候选路径代价应大于 0');

    % 近似回归检查：同种子、长预算
    cfg_reg = cfg;
    cfg_reg.time_budget_per_segment = 10.0;
    cfg_reg.max_iter_cap = 8000;

    rng(123);
    [old_conv, old_first, ~, ~] = get_Informed_RRT_Star_path_monitored_legacy(start_pos, goal_pos, obstacles, cfg_reg);
    rng(123);
    pool_reg = get_Informed_RRT_Star_path_pool(start_pos, goal_pos, obstacles, cfg_reg);
    assert((~isempty(old_conv) || ~isempty(old_first)) && ~isempty(pool_reg), 'T3 回归失败：新旧路径为空');

    % 回归参考优先使用 legacy 的收敛路径（更接近 pool 的代价排序语义）
    if ~isempty(old_conv)
        ref_path = old_conv;
    else
        ref_path = old_first;
    end

    % 不固定比较 pool_reg(1)：在候选池中寻找最匹配（或任一通过）路径
    ok_reg = false;
    best_idx = 1;
    best_score = inf;
    reg_details = struct('mean_point_dist', inf, 'max_point_dist', inf);
    for i = 1:numel(pool_reg)
        [ok_i, det_i] = compare_paths_approx(ref_path, pool_reg(i).path, cfg_reg);
        score_i = det_i.mean_point_dist + det_i.max_point_dist;
        if score_i < best_score
            best_score = score_i;
            best_idx = i;
            reg_details = det_i;
        end
        if ok_i
            ok_reg = true;
            best_idx = i;
            reg_details = det_i;
            break;
        end
    end
    assert(ok_reg, sprintf('T3 回归失败：候选池均未通过（best_idx=%d, mean=%.4f, max=%.4f）', ...
        best_idx, reg_details.mean_point_dist, reg_details.max_point_dist));

    fprintf('[PASS] T3 RRT 候选池与近似回归通过\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('[FAIL] T3 RRT 候选池/回归失败: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% T4: adaptive_hdi（危险区更密）
try
    rrt_path = [0, 5, 10, 15, 20; 0, 0, 0, 0, 0];
    obstacles = [10, 0, 1.8, 0, 0, 1];
    % 仅用于测试：放大 via 上限，避免裁剪导致危险/安全样本不足
    cfg_hdi = cfg;
    cfg_hdi.via_cap_factor = 10.0;
    [dense, ~, is_danger] = adaptive_hdi(rrt_path, obstacles, cfg_hdi, 1, 80, 0.1);
    assert(~isempty(dense), 'T4 失败：adaptive_hdi 输出为空');

    idx_d = find(is_danger);
    idx_s = find(~is_danger);
    assert(numel(idx_d) >= 3 && numel(idx_s) >= 3, 'T4 失败：危险/安全样本点不足，无法比较密度');

    d_spacing = mean(abs(diff(dense(1, idx_d))));
    s_spacing = mean(abs(diff(dense(1, idx_s))));
    assert(d_spacing < s_spacing, 'T4 失败：危险区点间距应小于安全区');

    fprintf('[PASS] T4 自适应 HDI 密度断言通过\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('[FAIL] T4 自适应 HDI 失败: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% T7: run_benchmark
try
    cfg_b = make_test_cfg();
    cfg_b.n_trials = 5;
    cfg_b.seeds = 1001:1005;
    cfg_b.write_csv = false;

    results = run_benchmark(cfg_b);
    assert(height(results) == 5, 'T7 失败：结果行数应为 5');
    assert(all(results.seed == cfg_b.seeds'), 'T7 失败：seed 列与输入不一致');
    assert(~any(isnan(double(results.success))), 'T7 失败：success 存在 NaN');

    fprintf('[PASS] T7 批量入口断言通过\n');
    pass_count = pass_count + 1;
catch ME
    fprintf('[FAIL] T7 批量入口失败: %s\n', ME.message);
    fail_count = fail_count + 1;
end

%% 总结
fprintf('\n===== 测试结束 =====\n');
fprintf('通过: %d\n', pass_count);
fprintf('失败: %d\n', fail_count);

if fail_count == 0
    fprintf('ALL_TESTS_PASS\n');
else
    error('test_framework:FAILED', '存在失败项，请查看上方日志。');
end

%% =========================================================================
% 本地测试配置（轻量）
%% =========================================================================
function cfg = make_test_cfg()
cfg = struct();

cfg.demoLen = 80;
cfg.demo_dt = 0.1;
cfg.alpha = 0.1;
cfg.N_C = 20;
cfg.maxIter_fcm = 12;
cfg.maxIter_gk = 12;
cfg.dt_repro = 0.02;

cfg.num_obs_target = 18;
cfg.min_gap = 1.0;
cfg.env_seed = 2025;
cfg.max_env_tries = 8000;
cfg.safe_margin = 1.5;
cfg.rrt_inflation = 1.2;

cfg.step_size = 1.5;
cfg.r_neighbor = 4.0;
cfg.max_iter_cap = 3000;
cfg.time_budget_per_segment = 0.2;
cfg.goal_sample_prob = 0.2;
cfg.K_candidates = 3;
cfg.rrt_quality_tolerance = 1.15;

cfg.rrt_cost_rel_tol = 0.03;
cfg.rrt_len_rel_tol = 0.03;
cfg.rrt_cmp_n = 200;
cfg.rrt_shape_mean_tol = 0.8;
cfg.rrt_shape_max_tol = 1.5;

cfg.base_interp_dist = 0.25;
cfg.min_interp_dist = 0.05;
cfg.max_interp_dist = 0.6;
cfg.hdi_k_risk = 2.0;
cfg.hdi_k_curvature = 1.5;
cfg.curvature_norm_ref = 1.0;
cfg.via_cap_factor = 2.0;

cfg.score_weights = [0.4, 0.2, 0.3, 0.1];
cfg.kappa_pass_thresh = 4.0;
cfg.pass_collision_margin = 1.0;
cfg.jerk_pass_thresh = 500;

cfg.seed = 1001;
cfg.n_trials = 5;
cfg.seeds = 1001:1005;
cfg.write_csv = false;
cfg.csv_path = fullfile('benchmark_results', 'test_benchmark.csv');
end
