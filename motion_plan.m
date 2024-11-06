function [x, y, theta, index] = motion_plan(curr_x, curr_y, curr_theta, r_plan, last_index, dist_threshold)
dist = hypot(r_plan(1, last_index) - curr_x, r_plan(2, last_index) - curr_y);
if dist < dist_threshold
    index = last_index + 1;
else
    index = last_index;
end

if index > size(r_plan, 2)
    index = -1;
end

if index == -1
    x = curr_x;
    y = curr_y;
    theta = curr_theta;
else
    x = r_plan(1, index);
    y = r_plan(2, index);
    theta = atan2(y - curr_y, x - curr_x);
end

theta = mod(theta, 2*pi);

end