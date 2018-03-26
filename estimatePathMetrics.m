function [angle, distance] = estimatePathMetrics(robotLocation, target, map, internalBorderSize)
    % This function calculates and returns the angle and distance between points point1(x1, y1) and point2(x2, y2).
    % Based on this, path estimation from start poistion to target position can be done.
    % Actually, we get path directions and distance informations from point to point.
    
    %% Path estimation from start position to target position
    internalBuffer = internalBoundaryBuffer(map, internalBorderSize);
    intermediateNodeCoordinates = pathPlan(robotLocation, target, internalBuffer);
    
    x1 = intermediateNodeCoordinates(1,1);
    x2 = intermediateNodeCoordinates(2,1);
    y1 = intermediateNodeCoordinates(1,2);
    y2 = intermediateNodeCoordinates(2,2);

    angle = atan2d(y2 - y1, x2 - x1) + 180;

    distance = sqrt (((y2 - y1) ^ 2) + ((x2 - x1) ^ 2));
    if distance > 10
        distance = distance / 2;
    end
        
end

