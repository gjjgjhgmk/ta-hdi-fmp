% =========================================================================
% MATLAB Baseline Export (Step 0 for ROS2 migration)
% Fixed-seed multi-run export for reproducible baseline comparison.
% =========================================================================
clear; clc;
addpath(pwd);
addpath(fullfile(pwd, 'func'));

cfg = get_baseline_cfg();
seeds = cfg.seed_start:(cfg.seed_start + cfg.n_trials - 1);

seed_col = zeros(cfg.n_trials, 1);
success_col = false(cfg.n_trials, 1);
path_len_col = nan(cfg.n_trials, 1);
min_dist_col = nan(cfg.n_trials, 1);
kappa_col = nan(cfg.n_trials, 1);
jerk_col = nan(cfg.n_trials, 1);
n_via_col = nan(cfg.n_trials, 1);
candidate_col = nan(cfg.n_trials, 1);

fprintf('\n===== MATLAB Baseline Export Start =====\n');
fprintf('n_trials = %d, seeds = [%d ... %d]\n', cfg.n_trials, seeds(1), seeds(end));

for i = 1:cfg.n_trials
    out = run_single_trial(cfg, seeds(i), false);
    seed_col(i) = seeds(i);
    success_col(i) = logical(out.success);
    path_len_col(i) = out.metrics.path_length;
    min_dist_col(i) = out.metrics.min_dist;
    kappa_col(i) = out.metrics.kappa_max;
    jerk_col(i) = out.metrics.jerk_rms;
    n_via_col(i) = out.n_via;
    candidate_col(i) = out.candidate_used;
    fprintf('[%02d/%02d] seed=%d success=%d min_dist=%.3f kappa=%.3f jerk=%.3f\n', ...
        i, cfg.n_trials, seeds(i), success_col(i), min_dist_col(i), kappa_col(i), jerk_col(i));
end

results = table(seed_col, success_col, path_len_col, min_dist_col, kappa_col, jerk_col, n_via_col, candidate_col, ...
    'VariableNames', {'seed', 'success', 'path_length', 'min_dist', 'kappa_max', 'jerk_rms', 'n_via', 'candidate_used'});

if ~exist(cfg.out_dir, 'dir')
    mkdir(cfg.out_dir);
end
writetable(results, cfg.out_csv);

fprintf('\nExport done: %s\n', cfg.out_csv);
fprintf('Success rate: %.2f%%\n', 100 * mean(success_col));
fprintf('===== MATLAB Baseline Export End =====\n\n');

function cfg = get_baseline_cfg()
cfg = struct();

% Core data / model config
cfg.demoLen = 200;
cfg.demo_dt = 0.1;
cfg.alpha = 0.1;
cfg.N_C = 40;
cfg.maxIter_fcm = 30;
cfg.maxIter_gk = 30;
cfg.dt_repro = 0.01;

% Env and collision
cfg.num_obs_target = 40;
cfg.min_gap = 1.0;
cfg.env_seed = 2025;
cfg.max_env_tries = 20000;
cfg.safe_margin = 1.5;
cfg.rrt_inflation = 1.2;

% RRT pool
cfg.step_size = 1.5;
cfg.r_neighbor = 4.0;
cfg.max_iter_cap = 8000;
cfg.time_budget_per_segment = 0.3;
cfg.goal_sample_prob = 0.2;
cfg.K_candidates = 3;
cfg.rrt_quality_tolerance = 1.15;

% T3 approx-regression defaults (kept for interface compatibility)
cfg.rrt_cost_rel_tol = 0.03;
cfg.rrt_len_rel_tol = 0.03;
cfg.rrt_cmp_n = 200;
cfg.rrt_shape_mean_tol = 0.8;
cfg.rrt_shape_max_tol = 1.5;

% HDI
cfg.base_interp_dist = 0.25;
cfg.min_interp_dist = 0.05;
cfg.max_interp_dist = 0.6;
cfg.hdi_k_risk = 2.0;
cfg.hdi_k_curvature = 1.5;
cfg.curvature_norm_ref = 1.0;
cfg.via_cap_factor = 2.0;

% T6 candidate score weights [length, curvature, risk, demo deviation]
cfg.score_weights = [0.4, 0.2, 0.3, 0.1];

% Verify
cfg.kappa_pass_thresh = 4.0;
cfg.pass_collision_margin = 1.0;  % NOTE: push_target_margin now follows this value (see push_traj_clear.m line 32)
cfg.jerk_pass_thresh = 500;
cfg.curvature_resample_n = 200;

% Push repair
cfg.enable_push_repair = true;
cfg.push_target_margin = 1.0;   % aligned with pass_collision_margin
cfg.push_max_iters = 3;
cfg.push_smooth_sigma = 5.0;

% Baseline export setup
cfg.n_trials = 30;
cfg.seed_start = 1001;
cfg.out_dir = fullfile('benchmark_results');
cfg.out_csv = fullfile(cfg.out_dir, 'matlab_baseline_30runs.csv');
end
