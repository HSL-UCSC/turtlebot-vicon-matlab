%--------------------------------------------------------------------------
% Hybrid Capture the Flag with Vicon Input
%--------------------------------------------------------------------------
% Description: This script implements a real-time Capture the Flag game
% using Vicon motion tracking data to track the actual positions of two
% robots (Cat as Blue Player, Mouse as Red Player). The game logic is
% adapted from HybridCaptureTheFlag.m.
%--------------------------------------------------------------------------

clear;
clc;
close all;

% -----------------------------
%%% Parameters & Initialization
% -----------------------------
DIST_THRESHOLD = 400; % Distance threshold for control
ARROW_SCALE = 400;    % Scale for visualization arrows
DT = 0.4;             % Time step

% Initialize Vicon Client
VICON_CLIENT = Vicon.Client();
VICON_CLIENT.destroy();
VICON_CLIENT.initialize();

% Define Vicon-tracked objects (Cat = Blue, Mouse = Red)
CAT_VICON = "Object1";  % Blue Player
MOUSE_VICON = "Object2"; % Red Player

CAT_PORT = 50804;
MOUSE_PORT = 50805;

% Initialize Robots
blue_player = turtlebot(VICON_CLIENT, CAT_VICON, CAT_PORT);
red_player = turtlebot(VICON_CLIENT, MOUSE_VICON, MOUSE_PORT);

% Controllers for each player
blue_controller = CustomPurePursuit_Control(DIST_THRESHOLD, blue_player.max_v, pi/4);
red_controller = CustomPurePursuit_Control(DIST_THRESHOLD, red_player.max_v, pi/4);

% Flag Positions
FB = [-1500, 0]; % Flag position for Blue Team
FR = [1500, 0];  % Flag position for Red Team
gf = 200;        % Radius of flag capture region

% Game State
Blue_Flag_Captured = false;
Red_Flag_Captured = false;

% Initialize figure for real-time plotting
figure(1);
clf;
hold on;
axis equal;
xlim([-2100 2100]);
ylim([-1100 1100]);
grid on;
xlabel('X');
ylabel('Y');

% Draw initial field boundaries
plot([-2000 2000 2000 -2000 -2000], [-1000 -1000 1000 1000 -1000], 'k', 'linewidth', 2);
plot([0 0], [-1000 1000], 'k--', 'linewidth', 2); % Midfield line

% Flag markers
BlueFlag = plot(FB(1), FB(2), 'bx', 'linewidth', 3, 'markersize', 10);
RedFlag = plot(FR(1), FR(2), 'rx', 'linewidth', 3, 'markersize', 10);

% Robot markers
blue_marker = plot(NaN, NaN, 'bo', 'MarkerFaceColor', 'blue', 'MarkerSize', 10);
red_marker = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'red', 'MarkerSize', 10);

% -----------------------------
%%% Game Loop
% -----------------------------
while true
    % Poll Vicon for actual positions
    [b_x, b_y, b_theta, blue_player] = blue_player.odom();
    [r_x, r_y, r_theta, red_player] = red_player.odom();
    
    % Check if players are out of bounds
    if any([b_y > 2400, b_y < -2700, b_x > 1900, b_x < -2000, ...
            r_y > 2400, r_y < -2700, r_x > 1900, r_x < -2000])
        disp("Player out of bounds! Game Over.");
        break;
    end
    
    % Check if Red captured the Blue flag
    if norm([r_x - FB(1), r_y - FB(2)]) < gf && ~Red_Flag_Captured
        disp("Red captured Blue flag!");
        Red_Flag_Captured = true;
    end
    
    % Check if Blue captured the Red flag
    if norm([b_x - FR(1), b_y - FR(2)]) < gf && ~Blue_Flag_Captured
        disp("Blue captured Red flag!");
        Blue_Flag_Captured = true;
    end

    % Check if Red scored (returned flag to base)
    if Red_Flag_Captured && norm([r_x - FR(1), r_y - FR(2)]) < gf
        disp("Red scored!");
        Red_Flag_Captured = false;
        RedFlag.Color = 'red';
    end

    % Check if Blue scored (returned flag to base)
    if Blue_Flag_Captured && norm([b_x - FB(1), b_y - FB(2)]) < gf
        disp("Blue scored!");
        Blue_Flag_Captured = false;
        BlueFlag.Color = 'blue';
    end

    % Defender behavior: Tagging
    if norm([b_x - r_x, b_y - r_y]) < gf
        disp("Blue tagged Red!");
        Red_Flag_Captured = false; % Red drops the flag
    end
    if norm([r_x - b_x, r_y - b_y]) < gf
        disp("Red tagged Blue!");
        Blue_Flag_Captured = false; % Blue drops the flag
    end

    % Controller updates (using real-time Vicon data)
    if ~Blue_Flag_Captured
        b_theta_target = atan2(FR(2) - b_y, FR(1) - b_x);
    else
        b_theta_target = atan2(FB(2) - b_y, FB(1) - b_x); % Return home with flag
    end
    blue_controller = blue_controller.update(b_x, b_y, b_theta, FR(1), FR(2), b_theta_target);
    [b_v, b_gamma, blue_controller] = blue_controller.get_control();
    blue_player = blue_player.drive(b_v, b_gamma, DT);

    if ~Red_Flag_Captured
        r_theta_target = atan2(FB(2) - r_y, FB(1) - r_x);
    else
        r_theta_target = atan2(FR(2) - r_y, FR(1) - r_x); % Return home with flag
    end
    red_controller = red_controller.update(r_x, r_y, r_theta, FB(1), FB(2), r_theta_target);
    [r_v, r_gamma, red_controller] = red_controller.get_control();
    red_player = red_player.drive(r_v, r_gamma, DT);

    % -----------------------------
    %%% Visualization Update
    % -----------------------------
    set(blue_marker, 'XData', b_x, 'YData', b_y);
    set(red_marker, 'XData', r_x, 'YData', r_y);

    % Update flag colors when captured
    if Red_Flag_Captured
        set(BlueFlag, 'Color', 'white'); % Blue flag taken
    else
        set(BlueFlag, 'Color', 'blue');  % Blue flag at base
    end
    if Blue_Flag_Captured
        set(RedFlag, 'Color', 'white'); % Red flag taken
    else
        set(RedFlag, 'Color', 'red');   % Red flag at base
    end

    % Quiver arrows for heading
    quiver(b_x, b_y, ARROW_SCALE*cos(b_theta), ARROW_SCALE*sin(b_theta), 'Color', 'blue', 'MaxHeadSize', ARROW_SCALE);
    quiver(r_x, r_y, ARROW_SCALE*cos(r_theta), ARROW_SCALE*sin(r_theta), 'Color', 'red', 'MaxHeadSize', ARROW_SCALE);
    
    % Redraw
    drawnow;
    
    % Pause for timing
    pause(DT);
end

% Stop both robots
blue_player.drive(0, 0, 0);
red_player.drive(0, 0, 0);