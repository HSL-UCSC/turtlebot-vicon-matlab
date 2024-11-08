clear;
clc;
close all;

DIST_THRESHOLD = 400;
ARROW_SCALE = 400;
DT = 0.4;

VICON_CLIENT = Vicon.Client();
VICON_CLIENT.destroy();
VICON_CLIENT.initialize();

ERLICH_VICON = "Object1";
ERLICH_PORT = 50804;

BACHMAN_VICON = "Object2";
BACHMAN_PORT = 50805;

mouse = turtlebot(VICON_CLIENT, ERLICH_VICON, ERLICH_PORT);
cat = turtlebot(VICON_CLIENT, BACHMAN_VICON, BACHMAN_PORT);
% controller = PID();
% controller = Basic_Control(car.max_v, car.max_gamma); % use this to sanity check car build
% controller = PurePursuit_Control(r_plan', 100, car.max_v, 4*car.max_gamma);
mouse_controller = CustomPurePursuit_Control(DIST_THRESHOLD, mouse.max_v, pi/4);
cat_controller = CustomPurePursuit_Control(DIST_THRESHOLD, cat.max_v, pi/4);

%%% END CODE HERE

% controller = PID_Control();

r_plan = matfile('plan_20_circle_mm.mat').data;
plot(r_plan(1,:),r_plan(2,:));

disp(r_plan(1, 1));
disp(r_plan(2, 1));

% index of the next point on the reference plan (starts as the first point)
index = 1;

recorded_data = [];
while true
    
    [m_x, m_y, m_theta, mouse] = mouse.odom();
    [c_x, c_y, c_theta, cat] = cat.odom();

    if (m_y > 2400 || m_y < -2700 || m_x > 1900 || m_x < -2000) || (c_y > 2400 || c_y < -2700 || c_x > 1900 || c_x < -2000)
        mouse.stop_car();
        cat.stop_car();
        disp("out of bounds");
        break
    end
    
    % find if the car is close enough to the next point on the reference plan
    [x_target, y_target, m_theta_target, index] = motion_plan(m_x, m_y, m_theta, r_plan, index, DIST_THRESHOLD);
    [m_done, mouse_controller] = mouse_controller.done();
    if index == -1 || m_done
        disp("circle done, restarting")
        index = 1;
    end

    % [c_done, cat_controller] = cat_controller.done();
    if abs(norm([c_x - m_x; c_y - m_y])) < DIST_THRESHOLD * 1.5
        disp("caught");
        break
    end
    
    % move the car to the next point on the reference plan
    mouse_controller = mouse_controller.update(m_x, m_y, m_theta, x_target, y_target, m_theta_target);
    [m_v, m_gamma, mouse_controller] = mouse_controller.get_control();
    mouse = mouse.drive(m_v, m_gamma, DT);

    c_theta_target = atan2(m_y - c_y, m_x - c_x);
    cat_controller = cat_controller.update(c_x, c_y, c_theta, m_x, m_y, c_theta_target);
    [c_v, c_gamma, cat_controller] = cat_controller.get_control();
    cat = cat.drive(c_v, c_gamma, DT);
    
    % record the data
    recorded_data = [recorded_data; c_x, c_y, c_theta, c_v, c_gamma, c_theta_target, m_x, m_y, m_theta, m_v, m_gamma, x_target, y_target, m_theta_target, index]; %#ok<AGROW>
    
    plot(r_plan(1,:), r_plan(2,:), '-o', 'Color', 'k');
    xlabel('Y')
    ylabel('X')
    ylim([-3000 3000])
    axis equal;
    hold on;
    % plot the car
    plot(x_target, y_target, '.', 'Color', 'r', 'MarkerSize', 20);
    quiver(m_x, m_y, ARROW_SCALE*cos(m_theta), ARROW_SCALE*sin(m_theta), 'Color', 'magenta', 'MaxHeadSize', ARROW_SCALE);
    quiver(m_x, m_y, ARROW_SCALE*cos(m_theta_target), ARROW_SCALE*sin(m_theta_target), 'Color', 'cyan', 'MaxHeadSize', ARROW_SCALE);
    
    quiver(c_x, c_y, ARROW_SCALE*cos(c_theta), ARROW_SCALE*sin(c_theta), 'Color', 'magenta', 'MaxHeadSize', ARROW_SCALE);
    quiver(c_x, c_y, ARROW_SCALE*cos(c_theta_target), ARROW_SCALE*sin(c_theta_target), 'Color', 'cyan', 'MaxHeadSize', ARROW_SCALE);

    plot(recorded_data(:,1), recorded_data(:,2), 'Color', 'b');
    plot(recorded_data(:,7), recorded_data(:,8), 'Color', "#EDB120")
    hold off;
    drawnow;
    
    pause(DT/2);
    
end
mouse.drive(0, 0, 0);
% heading & target heading over time
% distance from target position over time
% x over time
% y over time
% speed over time

% figure;
% subplot(2,1,1);
% plot(recorded_data(:,1));
% title('Car X');
% xlabel('Time');
% ylabel('X');
% 
% subplot(2,1,2);
% plot(recorded_data(:,2));
% title('Car Y');
% xlabel('Time');
% ylabel('Z');
% 
% figure;
% subplot(2,1,1);
% plot(recorded_data(:,8));
% title('Car Velocity');
% xlabel('Time');
% ylabel('Velocity');
% 
% subplot(2,1,2);
% plot(recorded_data(:,9));
% title('Car Steering Angle');
% xlabel('Time');
% ylabel('Steering Angle (rad)');
% 
% figure;
% plot(recorded_data(:,3));
% hold on;
% plot(recorded_data(:, 6));
% title("Heading vs. Target Heading");
% legend('Heading', 'Target Heading');
% xlabel('Time');
% ylabel('Heading (rad)');
% hold off;
% 
% figure;
% dist = abs(hypot(recorded_data(:,5)-recorded_data(:,2), recorded_data(:,4)-recorded_data(:,1)));
% plot(dist);
% title('Distance from Target Position');
% xlabel('Time');
% ylabel('Distance');