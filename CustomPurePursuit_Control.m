classdef CustomPurePursuit_Control
    properties
        x = 0;
        y = 0;
        theta = 0;
        v = 0; % Linear velocity
        gamma = 0; % Steering angle
        prediction = [];
        controller; % Instance of CustomPurePursuit
        waypoints;
        lastX = 0; % Previous X position
        lastY = 0; % Previous Y position
        DT = 0.4; % Fixed time step for the simulation
        reached_path = false;
        target_point = [];
        dist_threshold;
    end
    
    methods
        function obj = CustomPurePursuit_Control(dist_threshold, max_v, max_gamma, max_angular_v)
            % Initialize the custom pure pursuit controller with the desired parameters
            obj.controller = CustomPurePursuit('DesiredLinearVelocity', max_v, ...
                                               'MaxAngularVelocity', max_angular_v, ...
                                               'MaxSteeringAngle', max_gamma, ...
                                               'LookaheadDistance', dist_threshold);
            obj.dist_threshold = dist_threshold;
        end
        
        function obj = update(obj, x, y, theta, x_target, y_target, ~)
            % Update the current state and target for the controller
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            
            % Calculate speed based on the change in position over the fixed time step, DT
            dx = obj.x - obj.lastX;
            dy = obj.y - obj.lastY;
            speed = sqrt(dx^2 + dy^2) / obj.DT; % Speed is the magnitude of the velocity vector
            
            % Update last positions for the next speed calculation
            obj.lastX = x;
            obj.lastY = y;
            
            % Update the waypoints for the controller
%             obj.controller.Waypoints = [x_target, y_target]; % Set new target as waypoints
            
            % Explicitly call interpolateWaypoints if necessary
%             obj.controller = obj.controller.interpolateWaypoints(); % This line ensures waypoints are interpolated
            
            % if isempty(obj.first_point)
            %     obj.first_point = [x_target y_target];
            %     obj.controller.Waypoints = [x_target y_target];
            %     obj.controller = obj.controller.interpolateWaypoints();
            % elseif ~obj.reached_path && ~isequal(obj.first_point, [x_target y_target])
             if ~isequal(obj.target_point, [x_target y_target])
                obj.target_point = [x_target y_target];
                obj.controller.Waypoints = [x y; x_target y_target];
                obj.controller = obj.controller.interpolateWaypoints();
            end

            % Compute the control commands using the updated state and calculated speed
            [obj.v, obj.gamma, obj.controller] = obj.controller.control([x, y, theta], speed);
            
            % Compute and store the prediction or lookahead point for visualization or logging
            [~, lookaheadPoint, obj.controller] = obj.controller.findLookAheadPoint([x, y, theta]);
            obj.prediction = [obj.prediction; lookaheadPoint];

            % disp("v gamma")
            % disp(obj.v)
            % disp(obj.gamma);
        end
        
        function [v, gamma, obj] = get_control(obj)
            % Return the computed velocity and steering angle
            v = obj.v;
            gamma = obj.gamma;
        end

        function [out, obj] = done(obj)
            % out = (obj.controller.WaypointIndex == length(obj.controller.InterpolatedWaypoints)) && ...
            %     (abs(norm([obj.x, obj.y] - obj.controller.InterpolatedWaypoints(obj.controller.WaypointIndex, :))) < obj.dist_threshold);
            out = false;
        end
    end
end
