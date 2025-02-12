function [Boundary_defenders_pos, attackers_pos] = Positions_in_Matrix(MRpx, ORobots, NOA,ti)
% MRpx: x position of my robot
% ORobots: Opponent team's vector
% NOA: Number of robots in opponent team
% Check team:    
    fn = fieldnames(ORobots);
    Boundary_defenders_pos = []; % We append here the positions of the opponent robots
    attackers_pos = [];
    % If the opponent robot CORRECT THE FOLLOWING DESCRIPTION :
    % 1) is further to the boundary than my robot AND is in the same half as
    % my robot 
    % 2) has lost tagging ability, or
    % 3) is tagged,
    % its position value stored as in the opposite half so it is not accounted as an active
    % opponent
    % If it is further to the boundary than my robot AND is in the opposite half as
    % my robot, its position is multiplied by 100 because it is not a
    % threath
    for m=1:NOA
        z = fn{1+4*(m-1)}; % Extract the robot's position fieldname
        tau = fn{2+4*(m-1)}; % Extract the robot's tau fieldname
        q = fn{3+4*(m-1)}; % Extract the robot's q fieldname
        Boundary_defenders_pos = [Boundary_defenders_pos; 
            ([-100,0]*((MRpx*ORobots.(z)(ti,1))>0)*...
            ((norm(MRpx)>norm(ORobots.(z)(ti,1)))*((ORobots.(tau)(ti)>0)+ORobots.(q)(ti))+...
            (norm(MRpx)<norm(ORobots.(z)(ti,1))))+...
            [1 , 1]).*...
            ORobots.(z)(ti,1:2)];
        attackers_pos = [attackers_pos;% -sign(MRpx)*100*((norm(MRpx)<norm(ORobots.(z)(ti,1)))*(sign(MRpx*ORobots.(z)(ti,1))>0)+...
            %(ORobots.(tau)(ti)>0)+ORobots.(q)(ti))+ORobots.(z)(ti,1:2)];
            ([-100,0]*((MRpx*ORobots.(z)(ti,1))>0)*...
            ((norm(MRpx)<norm(ORobots.(z)(ti,1)))*((ORobots.(tau)(ti)>0)+ORobots.(q)(ti)))+...
            [1 , 1]).*...
            ORobots.(z)(ti,1:2)];
    end