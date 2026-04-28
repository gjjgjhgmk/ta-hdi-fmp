function score = score_path(rrt_path, demo_seg, obstacles, cfg)
% weighted scalar score. lower is better
if isempty(rrt_path) || size(rrt_path,2) < 2
    score = inf; return;
end

w = cfg.score_weights;

L = sum(sqrt(sum(diff(rrt_path,1,2).^2,1)));
L_straight = norm(rrt_path(:,1)-rrt_path(:,end));
J_len = L / max(L_straight, 1e-8);

kappa = compute_discrete_curvature(rrt_path);
if isempty(kappa)
    J_curv = 0;
else
    J_curv = prctile(kappa, 99);
end

ds = zeros(1, size(rrt_path,2));
for i = 1:size(rrt_path,2)
    ds(i) = min_point_to_obstacles(rrt_path(:,i), obstacles);
end
% Mixed risk: penalize local near-collision while keeping noise robustness.
J_risk = 0.7 / (min(ds) + 0.1) + 0.3 / (mean(ds) + 0.1);

if isempty(demo_seg)
    J_dev = 0;
else
    n = min(size(rrt_path,2), size(demo_seg,2));
    J_dev = mean(sqrt(sum((rrt_path(:,1:n)-demo_seg(:,1:n)).^2,1)));
end

J = [J_len, J_curv, J_risk, J_dev];
Jnorm = J ./ (J + 1);
score = w(1)*Jnorm(1) + w(2)*Jnorm(2) + w(3)*Jnorm(3) + w(4)*Jnorm(4);
end
