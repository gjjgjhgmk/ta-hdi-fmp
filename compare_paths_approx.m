function [ok, details] = compare_paths_approx(path_old, path_new, cfg)
% Approximate regression check for T3

cost_old = path_cost(path_old);
cost_new = path_cost(path_new);

cost_rel = abs(cost_new - cost_old) / max(cost_old, 1e-10);

res_old = resample_path_by_arclen(path_old, cfg.rrt_cmp_n);
res_new = resample_path_by_arclen(path_new, cfg.rrt_cmp_n);

d = sqrt(sum((res_old - res_new).^2, 1));
mean_d = mean(d);
max_d = max(d);

ok = (cost_rel <= cfg.rrt_cost_rel_tol) && ...
     (mean_d <= cfg.rrt_shape_mean_tol) && ...
     (max_d <= cfg.rrt_shape_max_tol);

details = struct(...
    'cost_old', cost_old, 'cost_new', cost_new, ...
    'cost_rel', cost_rel, ...
    'mean_point_dist', mean_d, 'max_point_dist', max_d);
end

function c = path_cost(path)
if isempty(path) || size(path,2) < 2
    c = inf; return;
end
c = sum(sqrt(sum(diff(path,1,2).^2,1)));
end

function out = resample_path_by_arclen(path, n)
if size(path,2) < 2
    out = repmat(path(:,1), 1, n);
    return;
end
s = [0, cumsum(sqrt(sum(diff(path,1,2).^2,1)))];
if s(end) < 1e-10
    out = repmat(path(:,1), 1, n);
    return;
end
sq = linspace(0, s(end), n);
out = zeros(2, n);
out(1,:) = interp1(s, path(1,:), sq, 'linear');
out(2,:) = interp1(s, path(2,:), sq, 'linear');
end
