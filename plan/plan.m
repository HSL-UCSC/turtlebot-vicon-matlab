clear;
close all;
% populate plan.mat

% create data points to form a circle to store in plan.mat
% circle is centered at (0,0) with radius 1.5
% circle is divided into 100 points

% create a vector of 100 points from 0 to 2*pi
theta = linspace(0,2*pi,20+1);
% plot(theta)

r = 1500*3/16;

% data = [cos(theta); sin(theta)].*1500;
data = [r * cos(theta); r * sin(theta) - 750];

% x = linspace(0, 2*pi, 60+1);
% data = [sin(x).*(2750/2)-(750/2); (x.*(3000 / (2*pi)))-1000];

plot(data(1,:),data(2,:), '-o')

save('../plan_20_circle_left_half.mat', 'data');