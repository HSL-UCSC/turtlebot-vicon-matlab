classdef Turtlebot
    properties
        Car_ID = "Object1";
        vicon_client;
        dr_client;
        
        %udp = udpport; % intializing udp protocol
        x = 0;
        y = 0;
        theta = 0;
        v;
        gamma;
        max_delta_theta = pi/16;
        wheel_base = 700;
        max_v = 200;
        min_v = 0;
        max_gamma = deg2rad(30);
        buf_size = 100;
        values;
        index = 1;
        buf_init_size = 0;
        filter_out = 0;
        node_1;
        pub;
        msg;

        reverse_driving = false;
    end
    methods
        function obj = Turtlebot()
            obj.vicon_client = Vicon.Client();
            obj.vicon_client.destroy();
            obj.vicon_client.initialize();

            %{
            obj.dr_client = py.awsdeepracer_control.Client(password="WThn8DOx", ip="128.114.59.181");
            obj.dr_client.set_manual_mode();
            obj.dr_client.start_car();
            
            obj.values = zeros(1, obj.buf_size);
            %}

            node_1=ros2node("node_1");
            pub=ros2publisher(node_1,"/erlich/cmd_vel","geometry_msgs/Twist")
            msg=ros2message(pub);
            msg.linear.x=0.0;
            msg.linear.y=0.0;
            msg.linear.z=0.0;
            msg.angular.x=0.0;
            msg.angular.y=0.0;
            msg.angular.z=0.0;
        end
        
        function obj = drive(obj, v, gamma, ~)
            v = clip(v, obj.min_v, obj.max_v);
            gamma = clip(gamma, -obj.max_gamma, obj.max_gamma);

            obj.v = v;
            obj.gamma = gamma;

            [v gamma]

            v = v * -1.0 / 1000.0;
            gamma = gamma * -1.0 / obj.max_gamma;

            if obj.reverse_driving
                v = -v;
                gamma = -gamma;
            end

            % v = typecast(v, 'uint8');
            % gamma = typecast(gamma, 'uint8');

            [v gamma]
            
            % send command to car
            msg.linear.x=v;
            msg.angular.z=gamma;
            send(pub,msg);
            % write(obj.udp, [v gamma], 'uint8', '128.114.59.181', 8888);
        end
        
        function [x, y, theta, obj] = odom(obj)
            pose = obj.vicon_client.get_pose(obj.Car_ID, obj.Car_ID);
            % these might be switched up
            obj.x = double(pose.translation{1});
            obj.y = double(pose.translation{2});

            t = double(pose.rotation{3});
            if obj.reverse_driving
                t = t + pi;
            end

            obj.theta = mod(t, 2*pi);
            
            if (obj.theta > pi)
                obj.theta = obj.theta - 2*pi;
            end
            
            x = obj.x;
            y = obj.y;
            theta = obj.theta;
        end
        
        function obj = weighted_low_pass_filter(obj, input)
            obj.values(obj.index) = input;
            obj.index = mod(obj.index, obj.buf_size)+1;
            obj.buf_init_size = min([obj.buf_init_size + 1, obj.buf_size]);
            
            obj.filter_out = mean(obj.values(1:obj.buf_init_size));
            disp(obj.index)
        end
    end
end