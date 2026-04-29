addpath('func');
cfg = get_default_cfg();
result = run_single_trial(cfg, 2025, false);
fprintf('Success: %d\n', result.success);
fprintf('min_dist: %.3f\n', result.metrics.min_dist);
fprintf('kappa_max: %.3f\n', result.metrics.kappa_max);
save('matlab_result.mat', 'result');
