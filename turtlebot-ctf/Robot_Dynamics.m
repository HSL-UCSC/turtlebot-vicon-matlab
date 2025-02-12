function [z_plus] = Robot_Dynamics(z,u,dt)%,PARAM)
    % Euler 1st order approximation of continuous evolution
    % Constant Speed
    
    u1 = u(1);
    u2 = u(2);

    p1 = z(1);
    p2 = z(2);
    theta = z(3);

    p1_plus = p1+u1*cos(u2)*dt;%p1+u1*cos(theta)*dt;
    p2_plus = p2+u1*sin(u2)*dt;%p2+u1*sin(theta)*dt;
    theta_plus = u(2);%theta+u(2)*dt; % TO FIX: TIMING

    z_plus = [p1_plus,p2_plus,theta_plus];
end