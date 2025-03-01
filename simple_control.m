%Button
fig = uifigure;
b = uibutton(fig, "state", "Position",[100,100,50,50],"Text","Apple");
start = uibutton(fig, "state", "Position",[200 100 50 50]);

node_1=ros2node("node_1");
pub=ros2publisher(node_1,"/bachman/cmd_vel","geometry_msgs/Twist")
pub0=ros2publisher(node_1,"/erlich/cmd_vel","geometry_msgs/Twist")
msg=ros2message(pub);
msg.linear.x=0.1;
msg.linear.y=0.0;
msg.linear.z=0.0;
msg.angular.x=0.0;
msg.angular.y=0.0;
msg.angular.z=0.0;

for c=1:1000
    send(pub,msg);
    send(pub0,msg);
    pause(0.1);
    msg.angular.z=sin(c/10);
end
