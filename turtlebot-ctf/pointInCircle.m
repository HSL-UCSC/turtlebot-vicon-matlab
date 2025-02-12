function isInCircle = pointInCircle(p, center, radius)
    % x, y are the coordinates of the point
    x = p(1);
    y = p(2);
    % center is a 2-element vector [center_x, center_y] defining the center of the circle
    % radius is the radius of the circle
    
    % Calculate the distance between the point and the circle center
    distance = sqrt((x - center(1))^2 + (y - center(2))^2);
    
    % Check if the distance is less than or equal to the radius
    isInCircle = (distance <= radius);
end