function out = run_single_trial(cfg, seed, do_plot)
% Single-trial runner for TA-HDI-FMP framework layer.

if nargin < 2
    seed = cfg.seed;
end
if nargin < 3
    do_plot = false;
end

rng(seed);
addpath('func');

demoLen = cfg.demoLen;
demo_dt = cfg.demo_dt;
alpha = cfg.alpha;
x_line = linspace(0, 100, demoLen);
y_line = 50 + 30 * sin(x_line * pi / 50);
my_demos{1}.pos = [x_line; y_line];

[Data, demo_dura] = demo_processing(my_demos, demoLen, demo_dt, alpha);
[C_x, pInvCov_x, p1_u_x] = fuzzymodellingCandGK(Data(1:3, :), demoLen, cfg.N_C, cfg.maxIter_fcm, cfg.maxIter_gk);
[C_y, pInvCov_y, p1_u_y] = fuzzymodellingCandGK(Data([1 2 4], :), demoLen, cfg.N_C, cfg.maxIter_fcm, cfg.maxIter_gk);

timeinput = cfg.dt_repro:cfg.dt_repro:(demoLen * demo_dt);
N_Data = numel(timeinput);

% Generate env
obstacles = zeros(cfg.num_obs_target, 6);
rng(cfg.env_seed);
count = 0; tries = 0;
while count < cfg.num_obs_target && tries < cfg.max_env_tries
    tries = tries + 1;
    ox = 10 + 80 * rand(); oy = 10 + 80 * rand(); typ = randi([1,2]);
    if typ == 1
        r = 2 + 3*rand(); cr = r; newo = [ox, oy, r, 0, 0, 1];
    else
        w = 4 + 6*rand(); h = 2 + 5*rand(); th = rand()*pi;
        cr = sqrt((w/2)^2 + (h/2)^2); newo = [ox, oy, w, h, th, 2];
    end
    ov = false;
    for j=1:count
        if obstacles(j,6)==1
            er = obstacles(j,3);
        else
            er = sqrt((obstacles(j,3)/2)^2 + (obstacles(j,4)/2)^2);
        end
        if norm([ox,oy]-obstacles(j,1:2)) < (cr + er + cfg.min_gap)
            ov = true; break;
        end
    end
    if ~ov
        count = count + 1; obstacles(count,:) = newo;
    end
end
obstacles = obstacles(1:count,:);

% Danger
 danger = [];
for k = 1:size(obstacles,1)
    for j = 1:demoLen
        p = [x_line(j), y_line(j)];
        if point_to_obstacle_distance(p, obstacles(k,:)) < cfg.safe_margin
            danger = [danger, j]; %#ok<AGROW>
        end
    end
end
danger = unique(sort(danger));

segments = {};
if ~isempty(danger)
    cur = danger(1);
    for i = 2:numel(danger)
        if danger(i)-danger(i-1) > 10
            segments{end+1} = cur; %#ok<AGROW>
            cur = danger(i);
        else
            cur = [cur, danger(i)]; %#ok<AGROW>
        end
    end
    segments{end+1} = cur;
end

all_via_points = [];
all_via_times = [];
candidate_used = -1;
for s = 1:numel(segments)
    seg = segments{s};
    idx_start = max(1, seg(1)-4);
    idx_goal  = min(demoLen, seg(end)+4);
    ls = [x_line(idx_start), y_line(idx_start)];
    lg = [x_line(idx_goal), y_line(idx_goal)];

    pool = get_Informed_RRT_Star_path_pool(ls, lg, obstacles, cfg);
    if isempty(pool)
        continue;
    end

    demo_seg = [x_line(idx_start:idx_goal); y_line(idx_start:idx_goal)];
    sc = inf(1,numel(pool));
    for i = 1:numel(pool)
        sc(i) = score_path(pool(i).path, demo_seg, obstacles, cfg);
    end
    [~, ord] = sort(sc, 'ascend');
    ord = ord(1:min(cfg.K_candidates, numel(ord)));

    selected = false;
    for oi = 1:numel(ord)
        c = ord(oi);
        [dense, t_local, ~] = adaptive_hdi(pool(c).path, obstacles, cfg, idx_start, idx_goal, demo_dt);
        if isempty(dense)
            continue;
        end
        vp = [all_via_points, dense];
        vt = [all_via_times, t_local];
        vscaled = (demoLen*demo_dt*demo_dura) * (vt / (demoLen*demo_dt));
        yx = fuzregre_modulation_yout(timeinput,demo_dura,alpha,N_Data,cfg.N_C,C_x,pInvCov_x,p1_u_x,[1 2],[3],vscaled,vp(1,:),[1]);
        yy = fuzregre_modulation_yout(timeinput,demo_dura,alpha,N_Data,cfg.N_C,C_y,pInvCov_y,p1_u_y,[1 2],[3],vscaled,vp(2,:),[1]);
        % Apply push repair for candidate check too (consistent with final pass)
        if isfield(cfg, 'enable_push_repair') && cfg.enable_push_repair
            tmp = push_traj_clear([yx;yy], obstacles, cfg);
            yx = tmp(1,:); yy = tmp(2,:);
        end
        [pass, ~] = verify_trajectory([yx;yy], obstacles, cfg);
        if pass
            all_via_points = vp;
            all_via_times = vt;
            candidate_used = oi;
            selected = true;
            break;
        end
    end

    if ~selected
        c = ord(1);
        [dense, t_local, ~] = adaptive_hdi(pool(c).path, obstacles, cfg, idx_start, idx_goal, demo_dt);
        if ~isempty(dense)
            all_via_points = [all_via_points, dense];
            all_via_times = [all_via_times, t_local];
            if candidate_used < 0
                candidate_used = 0;
            end
        end
    end
end

if ~isempty(all_via_points)
    vscaled = (demoLen*demo_dt*demo_dura) * (all_via_times / (demoLen*demo_dt));
    yx = fuzregre_modulation_yout(timeinput,demo_dura,alpha,N_Data,cfg.N_C,C_x,pInvCov_x,p1_u_x,[1 2],[3],vscaled,all_via_points(1,:),[1]);
    yy = fuzregre_modulation_yout(timeinput,demo_dura,alpha,N_Data,cfg.N_C,C_y,pInvCov_y,p1_u_y,[1 2],[3],vscaled,all_via_points(2,:),[1]);
else
    yx = interp1(1:demoLen, x_line, linspace(1,demoLen,N_Data), 'linear');
    yy = interp1(1:demoLen, y_line, linspace(1,demoLen,N_Data), 'linear');
end

% --- Framework-layer safety repair: push penetrating points outward ---
% This compensates for FMP modulation's limited diversion capability
% (FMP can only pull trajectory toward via points within cluster bandwidth;
%  points between via times may remain on or near the demo path which
%  passes through obstacles). push_traj_clear operates on [yx;yy] directly
% without touching func/ internals.
if isfield(cfg, 'enable_push_repair') && cfg.enable_push_repair
    traj_raw = [yx; yy];
    traj_fixed = push_traj_clear(traj_raw, obstacles, cfg);
    yx = traj_fixed(1, :);
    yy = traj_fixed(2, :);
end

[pass, m] = verify_trajectory([yx;yy], obstacles, cfg);
out = struct('seed',seed,'success',pass,'metrics',m,'candidate_used',candidate_used,'n_via',size(all_via_points,2));

if do_plot
    figure('Name', 'TA-HDI-FMP Framework Output', 'Position', [100, 100, 980, 640], 'Color', 'w');
    hold on; grid on; axis equal; axis([-5 105 -5 105]);
    for k = 1:size(obstacles,1)
        type = obstacles(k,6);
        cx = obstacles(k,1); cy = obstacles(k,2);
        fc = [1 0.85 0.85]; ec = [0.8 0.45 0.45];
        if type == 1
            rectangle('Position', [cx-obstacles(k,3), cy-obstacles(k,3), obstacles(k,3)*2, obstacles(k,3)*2], ...
                'Curvature', 1, 'FaceColor', fc, 'EdgeColor', ec, 'LineWidth', 1.0);
        else
            w = obstacles(k,3); h = obstacles(k,4); theta = obstacles(k,5);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            corners = [-w/2, -w/2, w/2, w/2; -h/2, h/2, h/2, -h/2];
            rc = R * corners;
            patch(rc(1,:)+cx, rc(2,:)+cy, fc, 'EdgeColor', ec, 'LineWidth', 1.0);
        end
    end
    plot(x_line, y_line, 'k--', 'LineWidth', 2);
    plot(yx, yy, 'g-', 'LineWidth', 2.6);
    title('TA-HDI-FMP Framework Layer', 'FontSize', 14, 'FontWeight', 'bold');
    legend({'Nominal Demo', 'Modulated Trajectory'}, 'Location', 'southwest', 'FontSize', 11);
end
end
