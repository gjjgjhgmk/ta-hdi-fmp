% =========================================================================
% TA-HDI-FMP Framework Layer (T1-T7)
% Entry file: keep config + dispatch only.
% NOTE: Do not modify func/ bottom FMP functions in this script.
% =========================================================================
clear; clc; close all;
addpath('func');

cfg = get_default_cfg();

if strcmpi(cfg.run_mode, 'benchmark')
    results = run_benchmark(cfg); %#ok<NASGU>
else
    out = run_single_trial(cfg, cfg.seed, cfg.do_plot); %#ok<NASGU>
end

function cfg = get_default_cfg()
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

% T3 approx-regression defaults
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
% Research hyper-parameter for ablation studies.
cfg.score_weights = [0.4, 0.2, 0.3, 0.1];

% Verify
cfg.kappa_pass_thresh = 4.0;
cfg.pass_collision_margin = 1.0;  % NOTE: push_target_margin now follows this value (see push_traj_clear.m line 32)
cfg.jerk_pass_thresh = 500;
cfg.curvature_resample_n = 200;  % resample before kappa calc (fix speed-nonuniform artifact)

% Push repair: framework-layer obstacle clearance repair after FMP modulation
cfg.enable_push_repair = true;
cfg.push_target_margin = 1.0;   % aligned with pass_collision_margin
cfg.push_max_iters = 3;
cfg.push_smooth_sigma = 5.0;

% Run mode
cfg.run_mode = 'single'; % 'single' or 'benchmark'
cfg.seed = 1001;
cfg.n_trials = 5;
cfg.seeds = 1001:1005;
cfg.write_csv = false;
cfg.csv_path = fullfile('benchmark_results', 'benchmark_results.csv');
cfg.do_plot = true;
end
