classdef TBcat
    properties
        Car_ID = "Object1";
        vicon_client;
        dr_client;
        
        udp = udpport; % intializing udp protocol
        vm_ip = '128.114.59.174';
        vm_port = 50804;

        % x = 0;
        % y = 0;
        % theta = 0;
        % v;
        % gamma;
        % max_delta_theta = pi/16;
        % wheel_base = 700;
        % max_v = 1;
        % min_v = 0;
        % max_gamma = deg2rad(30);
        % buf_size = 100;
        % values;
        % index = 1;
        % buf_init_size = 0;
        % filter_out = 0;
        % node_1;
        % pub;
        % msg;

        node_1;
        pub;
        msg;
        max_v=1.0;
        min_v=0.0;
        max_gamma=1.0;
        min_gamma=-1.0;
        x=0;
        y=0;
        theta=0;
        v;
        gamma;

        reverse_driving = false;
    end
    methods
        function obj = TBcat()
            obj.vicon_client = Vicon.Client();
            obj.vicon_client.destroy();
            obj.vicon_client.initialize();

            % obj.node_1=ros2node("node_1");
            % obj.pub=ros2publisher(obj.node_1,"/erlich/cmd_vel","geometry_msgs/Twist");
            % obj.msg=ros2message(obj.pub);
            % obj.msg.linear.x=0.0;
            % obj.msg.linear.y=0.0;
            % obj.msg.linear.z=0.0;
            % obj.msg.angular.x=0.0;
            % obj.msg.angular.y=0.0;
            % obj.msg.angular.z=0.0;
        end
        
        function obj = drive(obj, v, gamma, ~)
            v = clip(v, obj.min_v, obj.max_v);
            gamma = clip(gamma, obj.min_gamma, obj.max_gamma);

            obj.v = 2*v;
            obj.gamma = (gamma-1)*4;

            if obj.gamma>0.5
                obj.v=0;
            end
            if obj.gamma<-0.5
                obj.v=0;
            end

            disp([obj.v, obj.gamma])

            obj.v = typecast(obj.v, 'double');
            obj.gamma = typecast(obj.gamma, 'double');

            % send command to car
            write(obj.udp, [obj.v, obj.gamma], 'double', obj.vm_ip, obj.vm_port);
            %send(obj.pub,obj.msg);
        end
        
        function [x, y, theta, obj] = odom(obj)
            pose = obj.vicon_client.get_pose(obj.Car_ID, obj.Car_ID);
            % these might be switched up
            obj.x = double(pose.translation{1});
            obj.y = double(pose.translation{2});

            t = double(pose.rotation{3});
            % if obj.reverse_driving
            %     t = t + pi;
            % end

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

        function obj = stop_car(obj)
             write(obj.udp, [0, 0], 'double', obj.vm_ip, obj.vm_port);
        end
    end
end