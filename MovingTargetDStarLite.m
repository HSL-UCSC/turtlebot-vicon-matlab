classdef MovingTargetDStarLite
    properties
        km          % Heuristic weight increment
        s_start     % Hunter's (car's) current position
        s_goal      % Target's (dog's) current position
        g           % g-values for each state (cost to come)
        rhs         % rhs-values for each state (one-step lookahead cost)
        openList    % Priority queue for nodes to explore (states and keys)
        obstacles   % List of obstacle positions
        obstacle_radius  % Radius around obstacle to avoid
        step_size   % Step size for movement
        num_directions % Number of directions to sample for movement
    end
    
    methods
        % Constructor
        function obj = MovingTargetDStarLite()
            obj.km = 0;
            obj.g = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.rhs = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.openList = [];
            obj.obstacles = zeros(0, 2);  % Initialize an empty obstacle list
            obj.obstacle_radius = 0.5;  % Example radius for obstacle avoidance
            obj.step_size = 0.5;  % Set the step size for movement
            obj.num_directions = 32;  % Number of directions to sample
        end
        
        % Heuristic function: Euclidean distance
        function h = heuristic(~, s1, s2)
            h = norm(s1 - s2);  % Euclidean distance
        end
        
        % Get g-value for a state (default to infinity if not present)
        function g_val = get_g(obj, state)
            state_str = mat2str(state);
            if isKey(obj.g, state_str)
                g_val = obj.g(state_str);
            else
                g_val = inf;  % Default to infinity
            end
        end
        
        % Get rhs-value for a state (default to infinity if not present)
        function rhs_val = get_rhs(obj, state)
            state_str = mat2str(state);
            if isKey(obj.rhs, state_str)
                rhs_val = obj.rhs(state_str);
            else
                rhs_val = inf;  % Default to infinity
            end
        end
        
        % Set g-value for a state
        function obj = set_g(obj, state, value)
            state_str = mat2str(state);
            obj.g(state_str) = value;
        end
        
        % Set rhs-value for a state
        function obj = set_rhs(obj, state, value)
            state_str = mat2str(state);
            obj.rhs(state_str) = value;
        end
        
        % Add obstacle positions (add once during initialization)
        function obj = addObstacle(obj, obstacle_pos)
            obj.obstacles = [obj.obstacles; obstacle_pos];  % Add new obstacle
        end
        
        % Update obstacle position in place (for dynamic obstacles)
        function obj = updateObstacle(obj, id, new_pos)
            obj.obstacles(id, :) = new_pos;  % Update position of obstacle at index id
        end

        % Check if a state is in an obstacle's area
        function inObstacle = isInObstacle(obj, state)
            inObstacle = false;  % Default to false if no obstacles are present
            if isempty(obj.obstacles)
                return;  % No obstacles, so no state is in an obstacle
            end
            % If there are obstacles, check each one
            for i = 1:size(obj.obstacles, 1)
                obstacle_pos = obj.obstacles(i, :);
                if norm(state - obstacle_pos) < obj.obstacle_radius
                    inObstacle = true;
                    break;
                end
            end
        end
        
        % Cost function between two states (taking obstacles into account)
        function c = cost(obj, s1, s2)
            if obj.isInObstacle(s2)
                c = inf;  % Infinite cost if the state is inside an obstacle
            else
                c = norm(s1 - s2);  % Euclidean distance otherwise
            end
        end
        
        % Calculate the key for a given state
        function key = calculateKey(obj, state)
            g_val = obj.get_g(state);
            rhs_val = obj.get_rhs(state);
            heuristic_val = obj.heuristic(state, obj.s_goal);
            key = [min(g_val, rhs_val) + heuristic_val + obj.km, min(g_val, rhs_val)];
        end
        
        % Get successors of a state (allowing movement in any direction)
        function successors = getSuccessors(obj, s)
            % Use persistent variable to store the successors array between calls
            persistent successors_array;
        
            % Initialize the successors array only once (if not already initialized)
            if isempty(successors_array) || size(successors_array, 1) ~= obj.num_directions
                successors_array = zeros(obj.num_directions, 2);  % Preallocate array
            end
            
            % Calculate the angle for each direction
            angle_step = 2 * pi / obj.num_directions;  % Divide full circle into equal parts
        
            % Update the successors array in place
            for i = 1:obj.num_directions
                angle = (i - 1) * angle_step;  % Calculate the angle for this direction
                dx = obj.step_size * cos(angle);   % Compute x component of the step
                dy = obj.step_size * sin(angle);   % Compute y component of the step
                successors_array(i, :) = s + [dx, dy];  % Update in place
            end
        
            % Return the updated persistent array as successors
            successors = successors_array;
        end

        % Update a state (recalculate g and rhs, manage open list)
        function obj = updateState(obj, u)
            % Update rhs for state u based on its predecessors
            if ~isequal(u, obj.s_goal)
                obj = obj.computeRHS(u);
            end

            % Remove u from the open list if it's there
            obj = obj.removeOpenList(u);
            
            % If g and rhs differ, insert u into open list
            if obj.get_g(u) ~= obj.get_rhs(u)
                obj = obj.insertOpenList(u, obj.calculateKey(u));
            end
        end
        
        % Compute rhs for a state u based on its successors
        function obj = computeRHS(obj, u)
            successors = obj.getSuccessors(u);
            min_rhs = inf;
            for i = 1:size(successors, 1)
                succ = successors(i, :);
                min_rhs = min(min_rhs, obj.get_g(succ) + obj.cost(u, succ));
            end
            obj = obj.set_rhs(u, min_rhs);
        end

        % Main pathfinding loop (updates paths)
        function obj = computePath(obj)
            while ~isempty(obj.openList)
                % Sort open list by keys and select the node with the smallest key
                [~, idx] = min(cellfun(@(x) x(1), obj.openList(:,2)));  % Min key
                u = obj.openList{idx, 1};  % Get the state to expand
                old_key = obj.openList{idx, 2};  % Its key
                new_key = obj.calculateKey(u);  % Recalculate its key
                
                if any(new_key < old_key)
                    obj = obj.insertOpenList(u, new_key);  % Update with new key
                elseif obj.get_g(u) > obj.get_rhs(u)
                    obj = obj.set_g(u, obj.get_rhs(u));  % Update g-value
                    obj = obj.removeOpenList(u);  % Remove from open list
                    successors = obj.getSuccessors(u);
                    for i = 1:size(successors, 1)
                        succ = successors(i, :);
                        obj = obj.updateState(succ);
                    end
                else
                    obj = obj.set_g(u, inf);  % Set g to infinity
                    successors = obj.getSuccessors(u);
                    for i = 1:size(successors, 1)
                        succ = successors(i, :);
                        obj = obj.updateState(succ);
                    end
                    obj = obj.updateState(u);
                end
            end
        end
        
        % Get the next step towards the target
        function nextStep = getNextStep(obj)
            successors = obj.getSuccessors(obj.s_start);
            min_g = inf;
            nextStep = obj.s_start;
            for i = 1:size(successors, 1)
                succ = successors(i, :);
                g_val = obj.get_g(succ);
                if g_val < min_g
                    min_g = g_val;
                    nextStep = succ;
                end
            end
        end
        
        % Run the algorithm (main entry point)
        function obj = run(obj)
            obj.km = obj.km + obj.heuristic(obj.s_start, obj.s_goal);  % Update heuristic weight
            obj = obj.computePath();
        end
        
        % Insert a state into the open list
        function obj = insertOpenList(obj, state, key)
            obj.openList = [obj.openList; {state, key}];
        end
        
        % Remove a state from the open list
        function obj = removeOpenList(obj, state)
            idx = find(cellfun(@(x) isequal(x, state), obj.openList(:,1)), 1);
            if ~isempty(idx)
                obj.openList(idx, :) = [];
            end
        end
    end
end