function dmin = min_point_to_obstacles(point, obstacles)
if isempty(obstacles)
    dmin = inf;
    return;
end
dmin = inf;
for k = 1:size(obstacles,1)
    dmin = min(dmin, point_to_obstacle_distance(point, obstacles(k,:)));
end
end
