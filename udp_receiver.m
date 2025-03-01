clear;
close all;
clc;

node_1 = ros2node("node_1");
pub1 = ros2publisher(node_1,"/erlich/cmd_vel","geometry_msgs/Twist");
msg1=ros2message(pub1);
msg1.linear.x=0.0;
msg1.linear.y=0.0;
msg1.linear.z=0.0;
msg1.angular.x=0.0;
msg1.angular.y=0.0;
msg1.angular.z=0.0;

pub2 = ros2publisher(node_1,"/bachman/cmd_vel","geometry_msgs/Twist");
msg2=ros2message(pub2);
msg2.linear.x=0.0;
msg2.linear.y=0.0;
msg2.linear.z=0.0;
msg2.angular.x=0.0;
msg2.angular.y=0.0;
msg2.angular.z=0.0;

u1=udpport("LocalHost","128.114.59.130","LocalPort",50809);

data1 = [];
data2 = [];

% erlich only = -1, both = 0, bachman only = 1
WHICH_BOTS = -1;

while 1<2
    if WHICH_BOTS <= 0
        data1=read(u1, 2, "double");
    end
    if WHICH_BOTS >= 0
        data2=read(u2, 2, "double");
    end
    % if length(data1) < 1 || length(data2) < 1
    %     continue
    % end

    if ~isempty(data1)
        msg1.linear.x=data1(1);
        msg1.angular.z=data1(2);
        send(pub1,msg1);
    end
    
    if ~isempty(data2)
        msg2.linear.x=data2(1);
        msg2.angular.z=data2(2);
        send(pub2,msg2);
    end
end

% while 1<2
% 
%     msg.linear.x=v;
%     msg.angular.z=gamma;
%     send(pub,msg);
% end