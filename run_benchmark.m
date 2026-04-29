function results = run_benchmark(cfg)
if isempty(cfg.seeds)
    seeds = cfg.seed + (0:cfg.n_trials-1);
else
    seeds = cfg.seeds;
end
n = numel(seeds);

seed_col = zeros(n,1);
success_col = false(n,1);
path_len_col = nan(n,1);
min_dist_col = nan(n,1);
kappa_col = nan(n,1);
jerk_col = nan(n,1);
n_via_col = nan(n,1);
candidate_col = nan(n,1);

for i = 1:n
    out = run_single_trial(cfg, seeds(i));
    seed_col(i) = out.seed;
    success_col(i) = logical(out.success);
    path_len_col(i) = out.metrics.path_length;
    min_dist_col(i) = out.metrics.min_dist;
    kappa_col(i) = out.metrics.kappa_max;
    jerk_col(i) = out.metrics.jerk_rms;
    n_via_col(i) = out.n_via;
    candidate_col(i) = out.candidate_used;
end

results = table(seed_col, success_col, path_len_col, min_dist_col, kappa_col, jerk_col, n_via_col, candidate_col, ...
    'VariableNames', {'seed','success','path_length','min_dist','kappa_max','jerk_rms','n_via','candidate_used'});

if isfield(cfg,'write_csv') && cfg.write_csv
    out_path = cfg.csv_path;
    out_dir = fileparts(out_path);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    writetable(results, out_path);
end
end
