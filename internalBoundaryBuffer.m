function bufferBoundary = internalBoundaryBuffer(map, internalBoundarySize)
    %This function returns a map with buffer boundaries. It uses internalBoundarySize to do this.

    %% Initializaing empty arrays for creating buffer boundaries 
    midpointX = zeros(1, size(map, 1));
    midpointY = zeros(1, size(map, 1));
    shiftedMidpointXA = zeros(1, size(map, 1));
    shiftedMidpointYA = zeros(1, size(map, 1));
    shiftedMidpointXB = zeros(1, size(map, 1));
    shiftedMidpointYB = zeros(1, size(map, 1));
    pointINX = zeros(1, size(map, 1));
    pointINY = zeros(1, size(map, 1));
    orientation = zeros(1, size(map, 1));
    bufferBoundary = zeros(size(map, 1), 2);

    %% Processing vertices from adjacent map boundaries
    for currentWall = 1:1:size(map, 1)
        x1 = map(currentWall, 1);
        y1 = map(currentWall, 2);
        if currentWall < size(map, 1)
            x2 = map(currentWall + 1, 1);
            y2 = map(currentWall + 1, 2);
        elseif currentWall == size(map, 1)
            x2 = map(1, 1);
            y2 = map(1, 2);
        end

        %calculating the mindpoints of the walls.
        midpointX(currentWall) = (x1 + x2) / 2;
        midpointY(currentWall) = (y1 + y2) / 2;

        %calculating the current oritentation of the map's wall.s
        orientation(currentWall) = atan((y2 - y1) / (x2 - x1));

        %adding the buffer midpoints of the boundaries to the current walls of the map
        shiftedMidpointXA(currentWall) = midpointX(currentWall) + internalBoundarySize * cos(orientation(currentWall) + pi / 2);
        shiftedMidpointYA(currentWall) = midpointY(currentWall) + internalBoundarySize * sin(orientation(currentWall) + pi / 2);
        shiftedMidpointXB(currentWall) = midpointX(currentWall) + internalBoundarySize * cos(orientation(currentWall) - pi / 2);
        shiftedMidpointYB(currentWall) = midpointY(currentWall) + internalBoundarySize * sin(orientation(currentWall) - pi / 2);

        %ensuring that the buffer boundaries are not on or outside the map walls.
        [INA, ONA] = inpolygon(shiftedMidpointXA(currentWall), shiftedMidpointYA(currentWall), map(:, 1), map(:, 2));
        [INB, ONB] = inpolygon(shiftedMidpointXB(currentWall), shiftedMidpointYB(currentWall), map(:, 1), map(:, 2));

        if INA == 1 && ONA == 0 && INB == 1 && ONB == 0 %if the buffer midpoints are inside the walls  
            currentState = [midpointX(currentWall); midpointY(currentWall)];
            currentTargetA = [shiftedMidpointXA(currentWall); shiftedMidpointYA(currentWall)];
            currentTargetB = [shiftedMidpointXB(currentWall); shiftedMidpointYB(currentWall)];
            %checking if the buffer midpoints are visible from the map'swall.
            isVisibleA = checkLOS(currentState, currentTargetA, map);
            isVisibleB = checkLOS(currentState, currentTargetB, map);
            %discarding any midpoints which don't have a direct LOS to the wall.
            if isVisibleA == 1
                pointINX(currentWall) = shiftedMidpointXA(currentWall);
                pointINY(currentWall) = shiftedMidpointYA(currentWall);
            elseif isVisibleB == 1
                pointINX(currentWall) = shiftedMidpointXB(currentWall);
                pointINY(currentWall) = shiftedMidpointYB(currentWall);
            end
        elseif INA == 1 && ONA == 0 %if buffer midpoint A is isnide but not on top of the wall.
            pointINX(currentWall) = shiftedMidpointXA(currentWall);
            pointINY(currentWall) = shiftedMidpointYA(currentWall);
        elseif INB == 1 && ONB == 0 %if inflated midpoint B is isnide but not on top of the wall.
            pointINX(currentWall) = shiftedMidpointXB(currentWall);
            pointINY(currentWall) = shiftedMidpointYB(currentWall);
        end
    end

    %% Calculating the vertices of the buffer mapss
    for currentWall = 1:1:size(map, 1)
        if currentWall < size(map, 1)
            xA = pointINX(currentWall);
            yA = pointINY(currentWall);
            orientationA = orientation(currentWall);
            xB = pointINX(currentWall + 1);
            yB = pointINY(currentWall + 1);
            orientationB = orientation(currentWall + 1);
        elseif currentWall == size(map, 1)
            xA = pointINX(currentWall);
            yA = pointINY(currentWall);
            orientationA = orientation(currentWall);
            xB = pointINX(1);
            yB = pointINY(1);
            orientationB = orientation(1);
        end
        bufferBoundary(currentWall,1) = xB + yB - yA - (xB - xA) * tan(orientationA) / cos(orientationB) * tan(orientationA) - sin(orientationB) * cos(orientationB);
        bufferBoundary(currentWall,2) = yB + yB - yA - (xB - xA) * tan(orientationA) / cos(orientationB) * tan(orientationA) - sin(orientationB) * sin(orientationB);
    end
end