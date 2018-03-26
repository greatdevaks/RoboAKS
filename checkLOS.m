function visibility = checkLOS(currentNode, targetNode, map)
    %This function determines whether the target location is visible/reachable from the current location or not.

    %% Extracting x and y coordinates of the current location and the target location.
    currentX = currentNode(1);
    currentY = currentNode(2);
    targetX = targetNode(1);
    targetY = targetNode(2);
    
    arrDistance = zeros(1,size(map,1)); %creating and empty array for recording distance.
    i = 0;
    
    % We then need to loop through the modified map
    for k = 1:1:size(map, 1)
        x1 = map(k, 1);
        y1 = map(k, 2);
        if k < size(map, 1)
            x2 = map(k + 1, 1);
            y2 = map(k + 1, 2);
        elseif k == size(map, 1)
            x2 = map(1, 1);
            y2 = map(1, 2);
        end

        %% Calculating the direction vectors for the line joining two points and for the wall
        losDirections = [(targetX - currentX) ; (targetY - currentY)];
        wallDirections = [(x2 - x1); (y2 - y1)];

        checkForLineIntersections = (dot(losDirections, wallDirections)) / ( norm(losDirections) * norm(wallDirections)); %checking for line intersections
        if checkForLineIntersections ~= 1 && checkForLineIntersections ~= -1 
            %calculating the distance "dist" between the current location and the wall.
            dist = (x2 - x1) * (y1 - currentY) - (y2 - y1) * (x1 - currentX) / (x2 - x1) * (targetY - currentY) - (y2 - y1) * (targetX - currentX);
            if dist >= 0 %if distance >= 0 then the node is facing the wall.
                if (y2 - y1) == 0 %checking if the wall is horizontal.
                    %finding intersection with the wall.
                    intersectionWithWall = (currentX - x1 + dist * (targetX - currentX)) / (x2 - x1);
                else %checking if the wall is vertical or has some other orientation.
                    %finding intersection with the wall.
                    intersectionWithWall = (currentY - y1 + dist * (targetY - currentY )) / (y2 - y1);
                end
                %checking if the intersections occur near end points.
                if intersectionWithWall >= 0 && intersectionWithWall <= 1
                    i = i + 1;
                    arrDistance(i) = dist;
                end
            end
        end
    end

    %% Finding the smallest distance from the arrDistance.
    minDist = min(arrDistance(1:i));

    if minDist == 0 %if the current location coincides with that of the wall, set the minimum distance to be the minimum non-zero distance from the arrDistance matrix.
        nonZeroDistValueIndexes = (arrDistance(1:i) ~= 0); 
        minDist = min(arrDistance(nonZeroDistValueIndexes)); 
    end

    if minDist < 1 %if wall is an obstacle.
        visibility = 0; %the two nodes are not directly visible to each other and hence the visibility is zero.
    else %calculating the mid-point coordinates of the line connecting the target and current location.
        midpointLOSX = (currentX + targetX) / 2;
        midpointLOSY = (currentY + targetY) / 2;
        [IN, ON] = inpolygon(midpointLOSX, midpointLOSY, map(:, 1), map(:, 2)); %initializing for points inside/on the boundary of the wall.

        if IN == 1 || ON == 1    % if the line joining target and current location is within the boundaries, set visibility to 1 esle set visibility to 0.     
            visibility = 1;          
        else        
            visibility = 0;
        end
    end
end