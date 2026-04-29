function d = point_to_obstacle_distance(point, obs)
% point: 2x1 or 1x2
% obs: [cx, cy, p3, p4, theta, type], type: 1 circle, 2 OBB
if size(point, 2) > 1
    p = point(:);
else
    p = point;
end
cx = obs(1); cy = obs(2);
type = obs(6);
if type == 1
    r = obs(3);
    d = norm([p(1)-cx, p(2)-cy]) - r;
else
    w = obs(3)/2; h = obs(4)/2; th = obs(5);
    dx = p(1)-cx; dy = p(2)-cy;
    lx = dx*cos(-th) - dy*sin(-th);
    ly = dx*sin(-th) + dy*cos(-th);
    qx = abs(lx) - w;
    qy = abs(ly) - h;
    outside = norm([max(qx,0), max(qy,0)]);
    inside = min(max(qx, qy), 0);
    d = outside + inside;
end
end
