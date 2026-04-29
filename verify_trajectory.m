function [pass, metrics] = verify_trajectory(traj, obstacles, cfg)
% traj: 2xN
% NOTE: traj may be densely sampled (e.g. 2000 pts from FMP output).
% Curvature is computed on an arc-length-resampled version to avoid
% numerical blow-up caused by near-duplicate points on the time-uniform
% parametric trajectory (speed non-uniformity → tiny |v1|,|v2| → kappa spike).
metrics = struct('min_dist', inf, 'kappa_max', inf, 'jerk_rms', inf, ...
                 'pass_no_penetration', false, 'pass_clearance', false, ...
                 'pass_collision', false, 'pass_curvature', false, 'pass_jerk', false, 'path_length', inf);

if isempty(traj) || size(traj,2) < 2
    pass = false;
    return;
end

% --- Collision: check on full dense trajectory ---
N = size(traj,2);
for i = 1:N
    metrics.min_dist = min(metrics.min_dist, min_point_to_obstacles(traj(:,i), obstacles));
end

collision_margin = cfg.safe_margin;
if isfield(cfg, 'pass_collision_margin')
    collision_margin = cfg.pass_collision_margin;
end
metrics.pass_no_penetration = metrics.min_dist > -1e-6;  % -1e-6 absorbs floating-point boundary noise
metrics.pass_clearance = metrics.min_dist > collision_margin;
metrics.pass_collision = metrics.pass_no_penetration && metrics.pass_clearance;

% --- Curvature: resample to fixed arc-length grid first ---
% This removes time-parameterization speed artifacts before curvature calc.
n_resample = 200;
if isfield(cfg, 'curvature_resample_n')
    n_resample = cfg.curvature_resample_n;
end
traj_for_kappa = resample_arc(traj, n_resample);
kappa = compute_discrete_curvature(traj_for_kappa);
if isempty(kappa)
    metrics.kappa_max = 0;
else
    metrics.kappa_max = prctile(kappa, 99);
end
metrics.pass_curvature = metrics.kappa_max < cfg.kappa_pass_thresh;

% --- Jerk: compute on resampled trajectory (consistent with kappa basis) ---
acc = diff(traj_for_kappa, 2, 2);
if size(acc,2) >= 2
    jerk = diff(acc, 1, 2);
    metrics.jerk_rms = sqrt(mean(sum(jerk.^2, 1)));
else
    metrics.jerk_rms = 0;
end
metrics.pass_jerk = metrics.jerk_rms < cfg.jerk_pass_thresh;

metrics.path_length = sum(sqrt(sum(diff(traj,1,2).^2,1)));
pass = metrics.pass_collision && metrics.pass_curvature && metrics.pass_jerk;
end

% -----------------------------------------------------------------------
function out = resample_arc(path, n)
% Arc-length uniform resample of a 2xN path to 2xn.
% Eliminates time-parameterization speed non-uniformity before curvature calc.
if size(path,2) < 2
    out = path;
    return;
end
diffs = sqrt(sum(diff(path,1,2).^2,1));
arc = [0, cumsum(diffs)];
total = arc(end);
if total < 1e-10
    out = repmat(path(:,1), 1, n);
    return;
end
s_uniform = linspace(0, total, n);
out = zeros(2, n);
out(1,:) = interp1(arc, path(1,:), s_uniform, 'linear');
out(2,:) = interp1(arc, path(2,:), s_uniform, 'linear');
end
