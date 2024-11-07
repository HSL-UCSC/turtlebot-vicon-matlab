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
car1 = turtlebot(VICON_CLIENT, ERLICH_VICON, ERLICH_PORT);
car2 = turtlebot(VICON_CLIENT, BACHMAN_VICON, BACHMAN_PORT);
% controller = PID();
% controller = Basic_Control(car.max_v, car.max_gamma); % use this to sanity check car build
% controller = PurePursuit_Control(r_plan', 100, car.max_v, 4*car.max_gamma);
controller1 = CustomPurePursuit_Control(DIST_THRESHOLD, car1.max_v, pi/4);
controller2 = CustomPurePursuit_Control(DIST_THRESHOLD, car2.max_v, pi/4);

%%% END CODE HERE

% reference plan (imported from plan.m --> plan.mat)
r_plan = matfile('plan_20_circle_mm.mat').data;
plot(r_plan(1,:),r_plan(2,:));

disp(r_plan(1, 1));
disp(r_plan(2, 1));

% index of the next point on the reference plan (starts as the first point)
index = 1;

recorded_data = [];
while true
    
    [x1, y1, theta1, car1] = car1.odom();
    [x2, y2, theta2, car2] = car2.odom();

    if (y1 > 2400 || y1 < -2700 || x1 > 1900 || x1 < -2000) || (y2 > 2400 || y2 < -2700 || x2 > 1900 || x2 < -2000)
        car1.stop_car()
        car2.stop_car()
        disp("out of bounds");
        break
    end
    
    % find if the car is close enough to the next point on the reference plan
    [x_target1, y_target1, theta_target1, index] = motion_plan(x1, y1, theta1, r_plan, index, DIST_THRESHOLD);
    [done, controller1] = controller1.done();
    if index == -1 || done
        disp("done")
        break;
    end

    x_target2 = -x_target1;
    y_target2 = -y_target1;
    theta_target2 = mod((theta_target2 + pi) + 2*pi);

    if theta_target2 > pi
        theta_target2 = theta_target2 - 2*pi;
    end
    
    % move the car to the next point on the reference plan
    controller1 = controller1.update(x1, y1, theta1, x_target1, y_target1, theta_target1);
    [v1, gamma1, controller1] = controller1.get_control();
    car1 = car1.drive(v1, gamma1, DT);

    controller2 = controller2.update(x2, y2, theta2, x_target2, y_target2, theta_target2);
    [v2, gamma2, controller2] = controller2.get_control();
    car2 = car2.drive(v2, gamma2, DT);
    % record the data
    recorded_data = [recorded_data; x1, y1, theta1, x_target1, y_target1, theta_target1, x2, y2, theta2, x_target2, y_target2, theta_target2, index, v1, gamma1, v2, gamma2]; %#ok<AGROW>

    plot(r_plan(1,:), r_plan(2,:), '-o', 'Color', 'k');
    xlabel('Y')
    ylabel('X')
    ylim([-3000 3000])
    axis equal;
    hold on;

    % plot the car
    plot(x_target1, y_target1, '.', 'Color', 'r', 'MarkerSize', 20);
    quiver(x1, y1, ARROW_SCALE*cos(theta1), ARROW_SCALE*sin(theta1), 'Color', 'magenta', 'MaxHeadSize', ARROW_SCALE);
    quiver(x1, y1, ARROW_SCALE*cos(theta_target1), ARROW_SCALE*sin(theta_target1), 'Color', 'cyan', 'MaxHeadSize', ARROW_SCALE);
    plot(recorded_data(:,1), recorded_data(:,2), 'Color', 'b');

    plot(x_target2, y_target2, '.', 'Color', 'h', 'MarkerSize', 20);
    quiver(x2, y2, ARROW_SCALE*cos(theta2), ARROW_SCALE*sin(theta2), 'Color', 'magenta', 'MaxHeadSize', ARROW_SCALE);
    quiver(x2, y2, ARROW_SCALE*cos(theta_target2), ARROW_SCALE*sin(theta_target2), 'Color', 'cyan', 'MaxHeadSize', ARROW_SCALE);
    plot(recorded_data(:,7), recorded_data(:,8), 'Color', 'b');
    hold off;
    drawnow;
    
    pause(DT/2);
    
end
car1.drive(0, 0, 0);