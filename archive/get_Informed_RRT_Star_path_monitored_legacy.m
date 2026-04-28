function [rrt_path_conv, rrt_path_first, t_first, t_conv] = get_Informed_RRT_Star_path_monitored_legacy(start_pos, goal_pos, obstacles, cfg)
% Legacy behavior adapter for approximate-regression reference

step_size = cfg.step_size;
r_neighbor = cfg.r_neighbor;
max_iter = cfg.max_iter_cap;
num_obs = size(obstacles, 1);

t_start_rrt = tic;
c_best = inf;
c_min = norm(goal_pos - start_pos);
if c_min < 1e-9
    rrt_path_conv = [start_pos(:), goal_pos(:)];
    rrt_path_first = rrt_path_conv;
    t_first = 0;
    t_conv = 0;
    return;
end

x_center = (start_pos + goal_pos) / 2;
dir = (goal_pos - start_pos) / c_min;
angle = atan2(dir(2), dir(1));
C_mat = [cos(angle), -sin(angle); sin(angle), cos(angle)];

tree = [start_pos, 0, 0];
goal_idx = -1;
t_first = 0;
rrt_path_first = [];
found_first = false;

x_min = min(start_pos(1), goal_pos(1)) - 20.0;
x_max = max(start_pos(1), goal_pos(1)) + 20.0;
y_min = min(start_pos(2), goal_pos(2)) - 20.0;
y_max = max(start_pos(2), goal_pos(2)) + 20.0;

for iter = 1:max_iter
    if c_best < inf
        val = max(c_best^2 - c_min^2, 0);
        r1 = c_best / 2;
        r2 = sqrt(val) / 2;
        L_mat = [r1, 0; 0, r2];
        r_rand = sqrt(rand());
        theta_rand = 2*pi*rand();
        x_ball = [r_rand*cos(theta_rand); r_rand*sin(theta_rand)];
        rand_node = (C_mat * L_mat * x_ball + x_center')';
    else
        if rand < cfg.goal_sample_prob
            rand_node = goal_pos;
        else
            rand_node = [x_min + rand()*(x_max-x_min), y_min + rand()*(y_max-y_min)];
        end
    end

    dist = sqrt((tree(:,1)-rand_node(1)).^2 + (tree(:,2)-rand_node(2)).^2);
    [~, nearest_idx] = min(dist);
    nearest_node = tree(nearest_idx, 1:2);

    theta_step = atan2(rand_node(2)-nearest_node(2), rand_node(1)-nearest_node(1));
    new_node = nearest_node + [step_size*cos(theta_step), step_size*sin(theta_step)];

    if in_collision_legacy(new_node, obstacles, num_obs, cfg.rrt_inflation)
        continue;
    end

    dist_to_all = sqrt((tree(:,1)-new_node(1)).^2 + (tree(:,2)-new_node(2)).^2);
    neighbor_indices = find(dist_to_all <= r_neighbor);

    best_parent_idx = nearest_idx;
    min_cost = tree(nearest_idx,4) + norm(new_node - nearest_node);

    for i = 1:length(neighbor_indices)
        idx = neighbor_indices(i);
        ctmp = tree(idx,4) + dist_to_all(idx);
        if ctmp < min_cost
            best_parent_idx = idx;
            min_cost = ctmp;
        end
    end

    new_idx = size(tree,1) + 1;
    tree(new_idx,:) = [new_node, best_parent_idx, min_cost];

    for i = 1:length(neighbor_indices)
        idx = neighbor_indices(i);
        c_via = min_cost + dist_to_all(idx);
        if c_via < tree(idx,4)
            tree(idx,3) = new_idx;
            tree(idx,4) = c_via;
        end
    end

    if norm(new_node - goal_pos) <= step_size
        c_goal = min_cost + norm(new_node - goal_pos);
        if c_goal < c_best
            c_best = c_goal;
            goal_idx = new_idx;
            if ~found_first
                t_first = toc(t_start_rrt);
                found_first = true;
                curr = goal_idx;
                pathf = goal_pos;
                while curr > 0
                    pathf = [tree(curr,1:2); pathf]; %#ok<AGROW>
                    curr = tree(curr,3);
                end
                rrt_path_first = pathf';
            end
        end
    end
end

t_conv = toc(t_start_rrt);
if ~found_first
    rrt_path_conv = [];
    return;
end
curr = goal_idx;
pathc = goal_pos;
while curr > 0
    pathc = [tree(curr,1:2); pathc]; %#ok<AGROW>
    curr = tree(curr,3);
end
rrt_path_conv = pathc';
end

function flag = in_collision_legacy(new_node, obstacles, num_obs, inflation)
flag = false;
for k = 1:num_obs
    type = obstacles(k, 6);
    cx = obstacles(k, 1); cy = obstacles(k, 2);
    if type == 1
        if norm(new_node - [cx, cy]) <= (obstacles(k,3) + inflation)
            flag = true; return;
        end
    else
        th = obstacles(k,5);
        hw = obstacles(k,3)/2 + inflation;
        hh = obstacles(k,4)/2 + inflation;
        dx = new_node(1)-cx; dy = new_node(2)-cy;
        lx = dx*cos(-th) - dy*sin(-th);
        ly = dx*sin(-th) + dy*cos(-th);
        if abs(lx) <= hw && abs(ly) <= hh
            flag = true; return;
        end
    end
end
end
