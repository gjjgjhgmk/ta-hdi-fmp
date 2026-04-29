function kappa = compute_discrete_curvature(path)
% path: 2xN
N = size(path, 2);
if N < 3
    kappa = zeros(1, 0);
    return;
end
kappa = zeros(1, N - 2);
for i = 2:N-1
    v1 = path(:, i) - path(:, i-1);
    v2 = path(:, i+1) - path(:, i);
    den = norm(v1) * norm(v2) * norm(v1 + v2);
    if den < 1e-10
        kappa(i-1) = 0;
    else
        cross_val = v1(1)*v2(2) - v1(2)*v2(1);
        kappa(i-1) = 2 * abs(cross_val) / den;
    end
end
end
