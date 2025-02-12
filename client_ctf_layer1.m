clear;
clc;
close all;

DIST_THRESHOLD = 200;
ARROW_SCALE = 400;
DT = 0.4;

VICON_CLIENT = Vicon.Client();
VICON_CLIENT.destroy();
VICON_CLIENT.initialize();

ERLICH_VICON = "Object1";
ERLICH_PORT = 50804;

BACHMAN_VICON = "Object2";
BACHMAN_PORT = 50805;

%%% CHANGE CODE HERE
% car = Model();
% car = OsoyooV2();
% car = Deepracer();
car = turtlebot(VICON_CLIENT, ERLICH_VICON, ERLICH_PORT);
% car = turtlebot(VICON_CLIENT, BACHMAN_VICON, BACHMAN_PORT);
% controller = PID();
% controller = Basic_Control(car.max_v, car.max_gamma); % use this to sanity check car build
% controller = PurePursuit_Control(r_plan', 100, car.max_v, 4*car.max_gamma);
controller = CustomPurePursuit_Control(DIST_THRESHOLD, car.max_v, pi/4);

%%% END CODE HERE

% reference plan (imported from plan.m --> plan.mat)
r_plan = matfile('plan_20_circle_mm.mat').data;
plot(r_plan(1,:),r_plan(2,:));

disp(r_plan(1, 1));
disp(r_plan(2, 1));

% index of the next point on the reference plan (starts as the first point)
% Find the southern-most point (minimum y-value)
[~, start_index] = min(r_plan(2, :));
x_start = r_plan(1, start_index);
y_start = r_plan(2, start_index);
theta_start = pi/2; % Facing north

% Find the northern-most point (maximum y-value)
[~, goal_index] = max(r_plan(2, :));
x_goal = r_plan(1, goal_index);
y_goal = r_plan(2, goal_index);
theta_goal = pi/2; % Facing north

recorded_data = [];

% Move the car to the starting position
disp("Moving to starting position...");
controller = controller.update(0, 0, 0, x_start, y_start, theta_start);
[v, gamma, controller] = controller.get_control();
while hypot(car.x - x_start, car.y - y_start) > DIST_THRESHOLD
    car = car.drive(v, gamma, DT);
    pause(DT/2);
end

% Stop at starting position, face north, and pause
car.drive(0, 0, 0);
disp("Arrived at starting position. Pausing for 2 seconds...");
pause(2);

while true
    
    [x, y, theta, car] = car.odom();

    if y > 2400 || y < -2700 || x > 1900 || x < -2000
        car.stop_car()
        disp("out of bounds");
        break
    end
    
    % find if the car is close enough to the next point on the reference plan
    % Move to goal (northern-most point)
    if hypot(x - x_goal, y - y_goal) > DIST_THRESHOLD
        x_target = x_goal;
        y_target = y_goal;
        theta_target = theta_goal;
    % Move back to starting position after reaching goal
    elseif hypot(x - x_start, y - y_start) > DIST_THRESHOLD
        x_target = x_start;
        y_target = y_start;
        theta_target = theta_start;
    else
        disp("Mission complete");
        break;
    end

    [done, controller] = controller.done();
    if index == -1 || done
        disp("done")
        break;
    end
    
    % move the car to the next point on the reference plan
    controller = controller.update(x, y, theta, x_target, y_target, theta_target);
    [v, gamma, controller] = controller.get_control();
    car = car.drive(v, gamma, DT);
    % record the data
    recorded_data = [recorded_data; x, y, theta, x_target, y_target, theta_target, index, v, gamma]; %#ok<AGROW>

    plot(r_plan(1,:), r_plan(2,:), '-o', 'Color', 'k');
    xlabel('Y')
    ylabel('X')
    ylim([-3000 3000])
    axis equal;
    hold on;

    % plot the car
    plot(x_target, y_target, '.', 'Color', 'r', 'MarkerSize', 20);
    quiver(x, y, ARROW_SCALE*cos(theta), ARROW_SCALE*sin(theta), 'Color', 'magenta', 'MaxHeadSize', ARROW_SCALE);
    quiver(x, y, ARROW_SCALE*cos(theta_target), ARROW_SCALE*sin(theta_target), 'Color', 'cyan', 'MaxHeadSize', ARROW_SCALE);
    plot(recorded_data(:,1), recorded_data(:,2), 'Color', 'b');
    hold off;
    drawnow;
    
    pause(DT/2);
    
end
car.drive(0, 0, 0);
% heading & target heading over time
% distance from target position over time
% x over time
% y over time
% speed over time

figure;
subplot(2,1,1);
plot(recorded_data(:,1));
title('Car X');
xlabel('Time');
ylabel('X');

subplot(2,1,2);
plot(recorded_data(:,2));
title('Car Y');
xlabel('Time');
ylabel('Z');

figure;
subplot(2,1,1);
plot(recorded_data(:,8));
title('Car Velocity');
xlabel('Time');
ylabel('Velocity');

subplot(2,1,2);
plot(recorded_data(:,9));
title('Car Steering Angle');
xlabel('Time');
ylabel('Steering Angle (rad)');

figure;
plot(recorded_data(:,3));
hold on;
plot(recorded_data(:, 6));
title("Heading vs. Target Heading");
legend('Heading', 'Target Heading');
xlabel('Time');
ylabel('Heading (rad)');
hold off;

figure;
dist = abs(hypot(recorded_data(:,5)-recorded_data(:,2), recorded_data(:,4)-recorded_data(:,1)));
plot(dist);
title('Distance from Target Position');
xlabel('Time');
ylabel('Distance');