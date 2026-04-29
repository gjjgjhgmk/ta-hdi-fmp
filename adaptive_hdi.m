function [dense_path, t_local, is_danger_mask] = adaptive_hdi(rrt_path, obstacles, cfg, idx_start, idx_goal, demo_dt)
% Adaptive HDI interpolation

dense_path = [];
is_danger_mask = [];
if isempty(rrt_path) || size(rrt_path,2) < 2
    t_local = [];
    return;
end
danger_margin = cfg.safe_margin * cfg.rrt_inflation;

kappa_all = compute_discrete_curvature(rrt_path);
for p = 1:(size(rrt_path,2)-1)
    pt1 = rrt_path(:,p);
    pt2 = rrt_path(:,p+1);
    mid = 0.5 * (pt1 + pt2);
    dmin = min_point_to_obstacles(mid, obstacles);
    risk = 1 / (dmin + 0.1);
    risk_norm = risk / (risk + 1);

    if isempty(kappa_all)
        kappa = 0;
    else
        kk = min(max(p-1,1), numel(kappa_all));
        kappa = kappa_all(kk);
    end
    kappa_norm = min(kappa / max(cfg.curvature_norm_ref, 1e-6), 1.0);

    interp_dist = cfg.base_interp_dist / (1 + cfg.hdi_k_risk*risk_norm + cfg.hdi_k_curvature*kappa_norm);
    interp_dist = min(max(interp_dist, cfg.min_interp_dist), cfg.max_interp_dist);

    ninterp = max(2, ceil(norm(pt2 - pt1) / interp_dist));
    xx = linspace(pt1(1), pt2(1), ninterp);
    yy = linspace(pt1(2), pt2(2), ninterp);

    seg_pts = [xx; yy];
    seg_danger = false(1, ninterp);
    for q = 1:ninterp
        seg_danger(q) = min_point_to_obstacles(seg_pts(:,q), obstacles) < danger_margin;
    end

    if p < size(rrt_path,2)-1
        dense_path = [dense_path, seg_pts(:,1:end-1)]; %#ok<AGROW>
        is_danger_mask = [is_danger_mask, seg_danger(1:end-1)]; %#ok<AGROW>
    else
        dense_path = [dense_path, seg_pts]; %#ok<AGROW>
        is_danger_mask = [is_danger_mask, seg_danger]; %#ok<AGROW>
    end
end

% via cap
N_via_cap = max(2, round(cfg.via_cap_factor * cfg.N_C));
if size(dense_path,2) > N_via_cap
    danger_idx = find(is_danger_mask);
    safe_idx = find(~is_danger_mask);
    keep = unique([1, danger_idx, size(dense_path,2)]);
    rest_cap = N_via_cap - numel(keep);
    if rest_cap > 0 && ~isempty(safe_idx)
        pick = round(linspace(1, numel(safe_idx), min(rest_cap, numel(safe_idx))));
        keep = unique([keep, safe_idx(pick)]);
    end
    keep = sort(keep);
    dense_path = dense_path(:, keep);
    is_danger_mask = is_danger_mask(keep);
end

if size(dense_path,2) < 2
    t_local = [];
    return;
end

t_start = idx_start * demo_dt;
t_goal = idx_goal * demo_dt;
dacc = [0, cumsum(sqrt(sum(diff(dense_path,1,2).^2,1)))];
if dacc(end) < 1e-10
    t_local = [];
    return;
end
t_local = t_start + (dacc / dacc(end)) * (t_goal - t_start);
end
