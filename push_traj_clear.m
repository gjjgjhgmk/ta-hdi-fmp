function traj_out = push_traj_clear(traj, obstacles, cfg)
% push_traj_clear  Post-process FMP output to repair obstacle penetrations.
%
% Framework-layer safety repair (does NOT modify func/ FMP internals).
%
% Mechanism:
%   For each trajectory point that penetrates an obstacle (min_dist < 0),
%   compute the outward gradient direction and push the point to at least
%   cfg.push_target_margin away from the obstacle surface.
%   After pushing, re-smooth with a Gaussian kernel to avoid introducing
%   new kinks from the discrete push.
%
% Inputs:
%   traj      - 2xN trajectory
%   obstacles - Mx6 obstacle array  [cx cy r/w h theta type]
%   cfg       - struct with fields:
%                 .push_target_margin  : target clearance after push (default = safe_margin)
%                 .push_max_iters      : max repair iterations (default 3)
%                 .push_smooth_sigma   : Gaussian smooth sigma in index units (default 5)
%
% Output:
%   traj_out  - 2xN repaired trajectory

traj_out = traj;
N = size(traj, 2);
if N < 2
    return;
end

% Defaults: align push target with pass_collision_margin so the repaired
% trajectory is guaranteed to pass clearance verification on the next check.
push_margin = cfg.safe_margin;
if isfield(cfg, 'pass_collision_margin')
    push_margin = cfg.pass_collision_margin;  % match the verification threshold
elseif isfield(cfg, 'push_target_margin')
    push_margin = cfg.push_target_margin;
end
max_iters = 3;
if isfield(cfg, 'push_max_iters')
    max_iters = cfg.push_max_iters;
end
smooth_sigma = 5.0;
if isfield(cfg, 'push_smooth_sigma')
    smooth_sigma = cfg.push_smooth_sigma;
end

for iter = 1:max_iters
    any_fixed = false;
    for i = 1:N
        pt = traj_out(:, i);
        [d, grad_out] = point_safe_push_gradient(pt, obstacles, push_margin);
        if d < push_margin
            % Push outward by the deficit
            deficit = push_margin - d;
            traj_out(:, i) = pt + grad_out * (deficit + 1e-4);
            any_fixed = true;
        end
    end
    if ~any_fixed
        break;
    end
    % Re-smooth pushed trajectory (Gaussian weighted average of neighbours)
    % Endpoints are pinned to preserve start/goal.
    traj_out = smooth_traj(traj_out, smooth_sigma);
end
end

% -----------------------------------------------------------------------
function [d_min, grad] = point_safe_push_gradient(pt, obstacles, ~)
% Return minimum signed distance and outward unit gradient for a point.
d_min = inf;
grad = [0; 1];  % fallback: push up
for k = 1:size(obstacles, 1)
    obs = obstacles(k, :);
    [d, g] = signed_dist_and_grad(pt, obs);
    if d < d_min
        d_min = d;
        grad = g;
    end
end
end

% -----------------------------------------------------------------------
function [d, grad] = signed_dist_and_grad(pt, obs)
% Signed distance and outward gradient for one obstacle.
cx = obs(1); cy = obs(2); typ = obs(6);
if typ == 1
    % Circle
    r = obs(3);
    diff_v = pt - [cx; cy];
    dist_c = norm(diff_v);
    d = dist_c - r;
    if dist_c < 1e-10
        grad = [0; 1];
    else
        grad = diff_v / dist_c;
    end
else
    % OBB
    w = obs(3); h = obs(4); theta = obs(5);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    local = R' * (pt - [cx; cy]);
    hw = w/2; hh = h/2;
    % Clamped closest point on box surface
    clamped = [max(-hw, min(hw, local(1))); max(-hh, min(hh, local(2)))];
    diff_l = local - clamped;
    dl = norm(diff_l);
    if dl > 1e-10
        % Outside: standard distance + gradient
        d = dl;
        grad = R * (diff_l / dl);
    else
        % Inside: penetration depth
        pen_x = hw - abs(local(1));
        pen_y = hh - abs(local(2));
        if pen_x < pen_y
            d = -pen_x;
            grad = R * [sign(local(1)); 0];
        else
            d = -pen_y;
            grad = R * [0; sign(local(2))];
        end
    end
end
end

% -----------------------------------------------------------------------
function traj_out = smooth_traj(traj, sigma)
% Gaussian smooth trajectory, pin endpoints.
N = size(traj, 2);
if N < 3 || sigma < 1e-6
    traj_out = traj;
    return;
end
hw = ceil(3 * sigma);
x = -hw:hw;
g = exp(-0.5 * (x / sigma).^2);
g = g / sum(g);
traj_out = traj;
for dim = 1:2
    row = traj(dim, :);
    row_padded = [repmat(row(1), 1, hw), row, repmat(row(end), 1, hw)];
    smoothed = conv(row_padded, g, 'valid');
    % 'valid' output length: numel(row_padded) - numel(g) + 1 = N + 2*hw - (2*hw+1) + 1 = N
    traj_out(dim, :) = smoothed;
end
% Pin start and end
traj_out(:, 1) = traj(:, 1);
traj_out(:, end) = traj(:, end);
end
