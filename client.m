clear;
clc;
close all;

%pe = pyenv(Version="C:\Users\espott\AppData\Local\Programs\Python\Python311\pythonw.exe");

DIST_THRESHOLD = 600;
ARROW_SCALE = 200;
DT = 0.4;

% reference plan (imported from plan.m --> plan.mat)
r_plan = matfile('plan_20_circle_mm.mat').data;
% r_plan = matfile('plan_60_sine_2.mat').data;
%r_plan = [300; 400]; % use this to go to a point

%%% CHANGE CODE HERE
% car = Model();
% car = OsoyooV2();
% car = Deepracer();
car = Turtlebot();
% controller = PID();
% controller = Basic_Control(car.max_v, car.max_gamma); % use this to sanity check car build
% controller = PurePursuit_Control(r_plan', 100, car.max_v, 4*car.max_gamma);
controller = CustomPurePursuit_Control(DIST_THRESHOLD, car.max_v, car.max_gamma, 4*car.max_gamma);

%%% END CODE HERE

% controller = PID_Control();


plot(r_plan(1,:),r_plan(2,:));

disp(r_plan(1, 1));
disp(r_plan(2, 1));

% index of the next point on the reference plan (starts as the first point)
index = 1;

recorded_data = [];
while true
    
    [x, y, theta, car] = car.odom();

    if y > 3000 || y < -3000 || x > 2000 || x < -2000
        car.dr_client.stop_car()
        disp("out of bounds");
        break
    end
    
    % find if the car is close enough to the next point on the reference plan
    [x_target, y_target, theta_target, index] = motion_plan(x, y, theta, r_plan, index, DIST_THRESHOLD);
    [done, controller] = controller.done();
    if index == -1 || done
        disp("done")
        break;
    end
    
    plot(r_plan(1,:), r_plan(2,:), '-o', 'Color', 'k');
    xlabel('Y')
    ylabel('X')
    ylim([-3000 3000])
    axis equal;
    hold on;
    
    % move the car to the next point on the reference plan
    controller = controller.update(x, y, theta, x_target, y_target, theta_target);
    [v, gamma, controller] = controller.get_control();
    car = car.drive(v, gamma, DT);
    % record the data
    recorded_data = [recorded_data; x, y, theta, x_target, y_target, theta_target, index, v, gamma]; %#ok<AGROW>
    
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