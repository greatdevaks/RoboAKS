function intermediateNodeCoordinates = pathPlan(robotLocation, target, internalBuffer)
    %This athPlan function implements Dijkstra's Shortest Path Algorithm to find the path from the robot's initial loaction to the given target location.

    initialNodeSet = zeros(size(internalBuffer, 1) + 2, 3); %creating an array to contain the initial set of nodes.

    %% This portion forms the set of nodes that have not been visited yet
    initialNodeSet(1, 1:2) = [robotLocation(1), robotLocation(2)];
    initialNodeSet(2:size(internalBuffer, 1) + 1, 1:2) = internalBuffer;
    initialNodeSet(size(internalBuffer, 1) + 2, 1:2) = [target(1), target(2)];

    initialNodeSet(:, 3) = Inf * ones(size(initialNodeSet, 1), 1); %initially, random distances are assigned to all the nodes.

    indexVisibleNodes = zeros(1, size(initialNodeSet, 1)); %checking which all nodes are reachable from the robot's initial location.

    visibleNodesSet =  zeros(size(initialNodeSet, 1), size(initialNodeSet,2), size(initialNodeSet, 1)); %initializing the array that will contain the set of visible nodes to zero.

    initialVisibleIndex = 0; %setting the initial visible index to be zero.

    for processEachNode = 1:size(initialNodeSet, 1)
        allNodes = initialNodeSet;    
        for nextTargetNode = 1:size(initialNodeSet, 1)
            currentProcessingNode = initialNodeSet(processEachNode, :);
            targetProcessingNode = initialNodeSet(nextTargetNode, :);
            isTargetVisible = checkLOS(currentProcessingNode, targetProcessingNode, internalBuffer); %checking is the target node is visible from the current node.
            if isTargetVisible == 1 %this means that the target is visible.
                initialVisibleIndex = initialVisibleIndex + 1;
                indexVisibleNodes(initialVisibleIndex) = nextTargetNode; %record the index of the visible node. 
                allNodes(nextTargetNode, 3) = sqrt((targetProcessingNode(1) - currentProcessingNode(1)) ^ 2 + (targetProcessingNode(2) - currentProcessingNode(2)) ^ 2);
            end
        end
        visibleNodesSet(:, :, processEachNode) = allNodes; %set of all visible nodes.
    end

    remaingingNodes = zeros(1, size(initialNodeSet, 1)); %array that contains all the unvisited nodes.
    for indexOfRemainingNodes = 1:size(initialNodeSet)
        remaingingNodes(indexOfRemainingNodes) = indexOfRemainingNodes;
    end

    dijkstraResult = visibleNodesSet(:, :, 1);

    dijkstraResult(:, 4) = 1; %initializing the starting node as one.

    nodeUnderProcessing = 1;
    remaingingNodes = setdiff(remaingingNodes, nodeUnderProcessing); %updating the unprocessed node set.

    while size(remaingingNodes, 2) > 0 %loop until target node is reached and keep updating the unprocessed node set.
        distanceMetrics = dijkstraResult(remaingingNodes, 3);
        [~, indexCurrentNodeBeingProcessed] = min(distanceMetrics); %only the second variable is used.

        idCurrentNodeBeingProcessed = remaingingNodes(indexCurrentNodeBeingProcessed); %process the current node and remove it from set of unprocessed nodes.
        remaingingNodes = setdiff(remaingingNodes, idCurrentNodeBeingProcessed);

        for unprocessedNodeIndex = 1:size(remaingingNodes,2)
            idTargetNode = remaingingNodes(unprocessedNodeIndex); %consider the next unprocessed node as the target node.

            if visibleNodesSet(idTargetNode, 3, idCurrentNodeBeingProcessed) < Inf %checking if the target node and the current nodes are visible or not.

                cumDistCurrentTarget = dijkstraResult(idCurrentNodeBeingProcessed, 3);
                distCurrentTarget = visibleNodesSet(idTargetNode, 3, idCurrentNodeBeingProcessed); %calculating the distance between the current node and the target node.
                newCumDist = cumDistCurrentTarget + distCurrentTarget;

                oldCumDistToTarget = dijkstraResult(idTargetNode, 3);

                if newCumDist < oldCumDistToTarget %choosing the optimal distance.
                    dijkstraResult(idTargetNode, 3) = newCumDist;    
                    dijkstraResult(idTargetNode, 4) = idCurrentNodeBeingProcessed;
                end
            end
        end
    end

    path = zeros(1, size(dijkstraResult, 1)); %the array to store the shortest path.

    shortestPathIndex = 1;
    path(shortestPathIndex) = size(initialNodeSet, 1);

    while path(shortestPathIndex) > 1 %it will go until all nodes in path are covered.
        shortestPathIndex = shortestPathIndex + 1;
        path(shortestPathIndex) = dijkstraResult(path(shortestPathIndex - 1), 4);
    end

    path = fliplr(path(1:shortestPathIndex)); %final path.

    intermediateNodeCoordinates(:, 1) = initialNodeSet(path, 1);
    intermediateNodeCoordinates(:, 2) = initialNodeSet(path, 2);
end 