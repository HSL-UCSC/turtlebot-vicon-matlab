clear;
close all;

% Constants
DIST_THRESHOLD = 600;  % Distance threshold for considering target reached
DT = 0.4;               % Time step for control updates
ARROW_SCALE = 0.2;      % Scale for visualizing car and dog orientation

% Initialize the car (hunter), dog (target), and obstacles
car = TBcat();  % Car object represents the hunter
dog = TBmouse();  % Dog object represents the target

% obstacle1 = Obstacle(2);  % Obstacle 1 (example: ID 2 in OptiTrack system)
% obstacle2 = Obstacle(3);  % Obstacle 2 (example: ID 3 in OptiTrack system)

% Initialize the CustomPurePursuit_Controller for the car
controller = CustomPurePursuit_Control(DIST_THRESHOLD, car.max_v, car.max_gamma, 4*car.max_gamma);

% Initialize the Moving Target D* Lite algorithm
pathPlanner = MovingTargetDStarLite();

% Add the obstacles once (only add at the start)
% pathPlanner = pathPlanner.addObstacle([0, 0]);  % Initial placeholder
% pathPlanner = pathPlanner.addObstacle([0, 0]);  % Initial placeholder

% Set up the visualization window
figure(1);
clf;
axh = axes;
hold(axh, 'on');
xlim([-3, 3]);
ylim([-3, 3]);
grid on;
axis equal;

% Main control loop
while true
    % Get the current position of the car and dog from the OptiTrack system
    [car_x, car_y, car_theta, car] = car.odom();   % Get car's current position and orientation
    [dog_x, dog_y, dog_theta, dog] = dog.odom();   % Get dog's current position and orientation
    
    % [obs1_x, obs1_y, ~, obstacle1] = obstacle1.odom();  % Obstacle 1 position
    % [obs2_x, obs2_y, ~, obstacle2] = obstacle2.odom();  % Obstacle 2 position

    % Break if tracking is lost
    if any(isnan([car_x, car_y, dog_x, dog_y]))
        disp('Lost tracking! Stopping...');
        break;
    end

    % Break if tracking is lost (with obstacles)
    % if any(isnan([car_x, car_y, dog_x, dog_y, obs1_x, obs1_y, obs2_x, obs2_y]))
    %     disp('Lost tracking! Stopping...');
    %     break;
    % end

    % Update obstacles with new positions (update in place)
    % pathPlanner = pathPlanner.updateObstacle(1, [obs1_x, obs1_y]);
    % pathPlanner = pathPlanner.updateObstacle(2, [obs2_x, obs2_y]);

    % Update the path planner with the latest positions
    pathPlanner.s_start = [car_x, car_y];  % Current car position [x, y]
    pathPlanner.s_goal = [dog_x, dog_y];   % Current dog position [x, y]

    % Compute the next move using the Moving Target D* Lite algorithm
    pathPlanner = pathPlanner.run();

    % Get the next step on the path (the target waypoint for the car)
    nextStep = pathPlanner.getNextStep();

    % Update the CustomPurePursuit controller with the current and target positions
    controller = controller.update(car_x, car_y, car_theta, nextStep(1), nextStep(2), dog_theta);

    % Get the control inputs (velocity and steering angle) from the controller
    [v, gamma, controller] = controller.get_control();

    % Move the car towards the target
    car = car.drive(v, gamma, DT);

    % Plot the current positions of the car, the dog, and the obstacles
    plot(car_x, car_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % Car position
    plot(dog_x, dog_y, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');  % Dog position
    
    % plot(obs1_x, obs1_y, 'kx', 'MarkerSize', 12, 'LineWidth', 2);  % Obstacle 1
    % plot(obs2_x, obs2_y, 'kx', 'MarkerSize', 12, 'LineWidth', 2);  % Obstacle 2
    
    quiver(car_x, car_y, cos(car_theta) * ARROW_SCALE, sin(car_theta) * ARROW_SCALE, 'r');
    quiver(dog_x, dog_y, cos(dog_theta) * ARROW_SCALE, sin(dog_theta) * ARROW_SCALE, 'b');
    drawnow;

    % Stop if the car reaches the target (within the distance threshold)
    if norm([car_x, car_y] - [dog_x, dog_y]) < DIST_THRESHOLD
        disp('Target caught!');
        break;
    end
end