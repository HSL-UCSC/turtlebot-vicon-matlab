function [resultMat, NSH] = multiplyFirstElement(MRpx, OPosMat)
    % MRpx: x position of my robot

    % Check the sign of the parameter 'MRpx'
    signNum = sign(MRpx);
    
    % Find rows with different sign in the first column
    ORobotsInOpHalf = 0;
    ORobotsInOpHalf = OPosMat(:, 1) * signNum < 0;
    
    % Multiply the first element of matching rows by 100
    OPosMat(ORobotsInOpHalf, 1) = OPosMat(ORobotsInOpHalf, 1) * 100;
    
    % Return the modified matrix
    resultMat = OPosMat;

    %Return number of robots in the same half
    NSH = size(OPosMat ,1)- sum(ORobotsInOpHalf);
end