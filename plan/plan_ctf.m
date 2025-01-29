clear;
close all;

% Define the circle properties
r = 1500 * 3 / 16; % Radius
center = [0; -750]; % Center point

% First edge point at theta = 0
theta = 0;
edge_point = center + [r * cos(theta); r * sin(theta)];

% Define the data with three points
data = [center, edge_point, center];

% Plot the points
plot(data(1,:), data(2,:), '-o')
hold on;
plot(center(1), center(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Mark the center
hold off;

% Save to .mat file
save('../plan_3_points.mat', 'data');
