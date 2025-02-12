function [theta] = Angle_Control(ti, MRobots, ORobots, z, tau, q, eta, MF, OF, NMA, NOA, dt)
    % ti: actual time
    % MRobots: My Robots vector
    % ORobots: Opponent Robots vector
    % z, tau, q, eta: My state
    % MF: My flag
    % OF: Opponent's flag
    % NMA: Number of robots in my team
    % NOA: Number of robots in opponent team
    % dt: Steptime
    [Oponnent_Positions_0, Attackers_Positions] = Positions_in_Matrix(MRobots.(z)(ti,1), ORobots, NOA, ti); %Matrix NOAx2 where the positions of the NOA opponents are the rows 
    [Oponnent_Positions, active_opponents] = multiplyFirstElement(MRobots.(z)(ti,1), Oponnent_Positions_0);% Filter the robots that are in the same half
    [Attackers_Positions, active_attackers] = multiplyFirstElement(MRobots.(z)(ti,1), Attackers_Positions);% Filter the robots that are in the same half
    % Multiply the position of any robot that is not in the same half by a
    % 100 so it is not taken as the closest opponent (To be fixed)
    Closest_Opponent_index = Closest_Opponent(MRobots.(z)(ti,1:2),Oponnent_Positions)-1;
    Closest_Attacker_index = Closest_Opponent(MRobots.(z)(ti,1:2),Attackers_Positions)-1;
    fn = fieldnames(ORobots); 
    cz = fn{1+4*(Closest_Opponent_index-1)}; % Closest oponent in 'z_czB' or 'z_czR' format
    caz = fn{1+4*(Closest_Attacker_index-1)}; % Closest oponent in 'z_czB' or 'z_czR' format
    %
        if MRobots.(q)(ti) == 1 || (MRobots.(eta)(ti) == 1 && MRobots.(z)(ti,1)*MF(1)>= 0 )
            % Return to base controller R and B
            theta = calculateAngle(MRobots.(z)(ti,1),MRobots.(z)(ti,2),MF(1),MF(2));
        elseif  MRobots.(eta)(ti) == 1 && MRobots.(z)(ti,1)*MF(1)<= 0
            if active_opponents > 0
                % Return the flag (Avoid being intercepted by closest
                % defender that protects the boundary) R and B
                m_x = -(MRobots.(z)(ti,2)-ORobots.(cz)(ti,2))/(MRobots.(z)(ti,1)-ORobots.(cz)(ti,1));
                n_x = 0.5*(norm(MRobots.(z)(ti,1:2)-OF)^2-norm(ORobots.(cz)(ti,1:2)-OF)^2)/(MRobots.(z)(ti,1)-ORobots.(cz)(ti,1));
                yst = 40*sign(m_x);
                theta =...% (MF(1)<0)*pi+sign(MF(1))*acos((m_x*yst+n_x-(MRobots.(z)(ti,1)-OF(1)))/norm([m_x*yst+n_x,yst]-(MRobots.(z)(ti,1:2)-OF)));
                    sign(m_x)*(acos((m_x*yst+n_x-(MRobots.(z)(ti,1)-OF(1)))/norm([m_x*yst+n_x,yst]-(MRobots.(z)(ti,1:2)-OF))));
            else
                % Cross to their own half to return the flag (no threat of
                % being intercepted
                theta = (1+ sign(MRobots.(z)(ti,1)))*pi/2;
            end
        elseif  active_attackers+ active_opponents> 0  && MRobots.(eta)(ti) == 0 && MRobots.(q)(ti) == 0 && MRobots.(tau)(ti) <= 0 && MRobots.(z)(ti,1)*MF(1)> 0
            if MRobots.mu(ti) == 1 && active_attackers > 0
                % Defend the flag (Capture the closest attacker that aims to capture the flag)
                xst = 0.5*(norm(ORobots.(caz)(ti,1:2)-MF)^2-norm(MRobots.(z)(ti,1:2)-MF)^2)*(ORobots.(caz)(ti,1)-MRobots.(z)(ti,1))/(norm(ORobots.(caz)(ti,1:2)-MRobots.(z)(ti,1:2))^2);
                yst = 0.5*(norm(ORobots.(caz)(ti,1:2)-MF)^2-norm(MRobots.(z)(ti,1:2)-MF)^2)*(ORobots.(caz)(ti,2)-MRobots.(z)(ti,2))/(norm(ORobots.(caz)(ti,1:2)-MRobots.(z)(ti,1:2))^2);
               theta = acos((xst-MRobots.(z)(ti,1))/norm([xst,yst]-(MRobots.(z)(ti,1:2))));
            elseif  MRobots.mu(ti) == 0 && active_opponents > 0 %Optional: Find a way of finding who has the flag and go after him!!!
                % Defend the boundary when the flag has been captured
                m_x = -(ORobots.(cz)(ti,2)-MRobots.(z)(ti,2))/(ORobots.(cz)(ti,1)-MRobots.(z)(ti,1));
                n_x = 0.5*(norm(ORobots.(cz)(ti,1:2)-OF)^2-norm(MRobots.(z)(ti,1:2)-OF)^2)/(ORobots.(cz)(ti,1)-MRobots.(z)(ti,1));
                yst = 40*sign(m_x);
                theta = (MF(1)<0)*pi+sign(MF(1))*acos((m_x*yst+n_x-(MRobots.(z)(ti,1)-OF(1)))/norm([m_x*yst+n_x,yst]-(MRobots.(z)(ti,1:2)-OF)));
            else
                theta = sin(ti*dt*4); 
            end    
        elseif matches(z,'z_2B') && ORobots.mu(ti) == 1 
            if MRobots.(z)(ti,1)*MF(1)>0 
                % Cross to the opponent's half plane
                theta = (1+ sign(MRobots.(z)(ti,1)))*pi/2;
            else%if MRobots.(z)(ti,1)*MF(1)<0
                % Capture the flag of Red Team
                theta = calculateAngle(MRobots.(z)(ti,1),MRobots.(z)(ti,2),OF(1),OF(2));
            end
        elseif any([matches(z,'z_1R'), matches(z,'z_2R'),matches(z,'z_3R')])
            % Arbitrary Controller for every Robot of Red team
            theta = sin(ti*dt*4)+pi;
        else
            % Arbitrary Controller for the rest of the robots of Blue team
            theta = sin(ti*dt*4);
        end
    
end