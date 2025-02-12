function angle = calculateAngle(x1, y1, x2, y2)
    % Calculate the angle between two points (x1, y1) and (x2, y2)

    % Calculate the differences in x and y coordinates
    dx = x2 - x1;
    dy = y2 - y1;

    % Use atan2 to get the angle in radians
    angle = atan2(dy, dx);

    % Convert the angle to degrees
    % angle = rad2deg(angle);
end