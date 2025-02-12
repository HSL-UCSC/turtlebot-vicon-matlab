function [closestParticleIndex, closestParticleDistance]  = Closest_Opponent(MyPosition, Opponents_Positions)
    % The nx2 matrix representing particle positions
    Opponents_Positions = [MyPosition; Opponents_Positions];
    % Build kd-tree
    kdtree = createns(Opponents_Positions, 'NSMethod', 'kdtree');
    % Query for the nearest neighbor
    pointOfInterest = Opponents_Positions(1, :); % position of particle 1
    [idx, dist] = knnsearch(kdtree, pointOfInterest, 'K', 2);
    % idx(2) contains the index of the closest particle to particle 1
    closestParticleIndex = idx(2);
    % dist(2) contains the distance to the closest particle
    closestParticleDistance = dist(2);