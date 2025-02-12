function isInRectangle = pointInRectangle(p, rect)
    % x, y are the coordinates of the point
    % rect is a 4-element vector [x_min, y_min, x_max, y_max] defining the rectangle
    
    x = p(1);
    y = p(2);

    % Check if the point is within the rectangle boundaries
    isInRectangle = (x >= rect(1) && x <= rect(3) && y >= rect(2) && y <= rect(4));
end