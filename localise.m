function [botSim] = localise(botSim, map, target)
    %The "localise" function has three input parameters: botSim object, a map, and a target location. It returns a botSim object.

    %% basic initializations
    modifiedMap = map; %initializing the map
    botSim.setMap(modifiedMap); %setting up map for botSim

    numscan = 30; %specifying the number of scans to perform
    botSim.setScanConfig(botSim.generateScanConfig(numscan)); %specifying scan configuration for botSim

    %% Specifying basic details for Particle Filter
    maxNumOfIterations = 30; %specifies the maximum number of iterations
    numParticles = 300; %specifies the number of particles

    %*************iska kuch karo
    %botDemo is the estimated botSim
    [botSim, botDemo] = PFModule(botSim, modifiedMap, numParticles, maxNumOfIterations, numscan);    
    %*************iska kuch kar lenge

    if botSim.debug()
        distance = sqrt(sum((botSim.getBotPos() - botDemo.getBotPos()) .^ 2));
    end

    %% Path Planning
    internalBoundarySize = 6; %it is the size of the internal boundary

    %% Calculate new extended, internl borders
    bufferBoundary = internalBoundaryBuffer(map, internalBoundarySize); %this function draws internal boundary buffers in map.

    %% Plot new boundaries
    map_shifted_draw = bufferBoundary;
    map_shifted_draw(size(map_shifted_draw, 1) + 1, :) = bufferBoundary(1, :);

    if botSim.debug()
        plot(map_shifted_draw(:, 1), map_shifted_draw(:, 2), 'Color', 'cyan');
    end

    %% Path plotting

    errorNoise = 0.005; %initial value is 0.002

    robotLocation = botDemo.getBotPos();
    while robotLocation(1) > target(1) + errorNoise || robotLocation(1) < target(1) - errorNoise || robotLocation(2) > target(2) + errorNoise || robotLocation(2) < target(2) - errorNoise
        [angle, distance] = estimatePathMetrics(robotLocation, target, map, internalBoundarySize);
        robotHeading = botDemo.getBotAng();
        radianAngle = (angle) * 0.01745329251;    

        if botSim.debug()
            botSim.drawBot(3, 'red');
            botDemo.drawBot(3, 'cyan');
        end

        botSim.turn(pi - robotHeading + radianAngle);
        botDemo.turn(pi - robotHeading + radianAngle);

        botScan = botSim.ultraScan();

        %Run the particle filter again if the robot moves through a wall
        if botScan(1) <= distance
            [botSim, botDemo] = PFModule(botSim, modifiedMap,numParticles, maxNumOfIterations, numscan);
        else
            botSim.move(distance);
            botDemo.move(distance);
        end

        botScan = botSim.ultraScan();
        botDemoScan = botDemo.ultraScan();

        %calculate the difference between the demo robot and the real robot
        difference = (sum(botDemoScan - botScan) / numscan);
        threshold = 3;

        %Run the particle filter again if the difference is greater than the threshold
        if (abs(difference) > threshold)
            [botSim, botDemo] = PFModule(botSim, modifiedMap,numParticles, maxNumOfIterations, numscan);
        end

        %Estimated location of robot
        robotLocation = botDemo.getBotPos();

        if botSim.debug()
            pause(1);
        end
    end

    %% Drawing the botSim and botDemo
    if botSim.debug()
        hold off; %the drawing is cleard when the hold off function is called
        botSim.drawMap(); %drawing is enabled
        botSim.drawBot(25,'g'); %drawing robot
        botDemo.drawBot(25,'r'); %drawing the demo bot
        drawnow;
    end
    
end
