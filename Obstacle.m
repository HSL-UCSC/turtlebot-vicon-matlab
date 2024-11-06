classdef Obstacle
    properties
        dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
        HostIP = '127.0.0.1';
        Obstacle_ID;  % Rigid body ID of the obstacle from Motive
        theClient = nan;
        x = -Inf;
        y = -Inf;
        theta;
    end
    
    methods
        % Constructor for Obstacle
        function obj = Obstacle(id)
            obj.Obstacle_ID = id;
            obj.theClient = NatNetML.NatNetClientML(0);
            obj.theClient.Initialize(obj.HostIP, obj.HostIP);
        end
        
        % Method to retrieve obstacle's position
        function [x, y, theta, obj] = odom(obj)
            [ObstaclePos] = GetDronePosition(obj.theClient, obj.Obstacle_ID);
            x = double(ObstaclePos(2));  % Extract x position
            y = double(ObstaclePos(3));  % Extract y position
            theta = obj.theta;  % Theta may not be necessary for obstacles
            obj.x = x;
            obj.y = y;
        end
    end
end