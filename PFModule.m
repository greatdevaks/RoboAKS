function [botSim, botDemo] = PFModule(botSim, map, numParticles, maxNumOfIterations, numscan)
%This function implements the Particle Filter Algorithm and Localization concept.

% ****** generating some random particles inside the map ******
num = numParticles; %num is the number of particles
particles(num,1) = BotSim; %setting up a vector of objects/particles

for i = 1:num
    particles(i) = BotSim(map);  %every particle has access to the original map
    particles(i).randomPose(5); %particles are being populated 10cm from the wall
    particles(i).setScanConfig(particles(i).generateScanConfig(numscan)); %scan configuration is being set for each particle
    particles(i).setMotionNoise(2); %motion noise is being added
    particles(i).setTurningNoise(pi/10); %turning noise is being added
end
% ****** generation of random particles completed ******

n = 0; %n is just a counter variable to keep trac of the number of iterations done

while(n < maxNumOfIterations) %particle filter loop
    n = n + 1; %incrementing the counter
    botScan = botSim.ultraScan(); %ultraScan scans for the surrounding obstacles
    
    %% Code for updating the particle scans
    arrPartScan = zeros(numscan, num); %creates an array of size numscan*num (20*300) filled with all zeros
    arrDiff = zeros(numscan, num); %creates an array of size numscan*num (20*300) filled with all zeross; calculates the difference between the scan vetors
    arrWeight = zeros(num, 1); %creates an array of size num*1 (300*1) filled with all zeros; this will maintian weight vector of all particles
    particleWeight = zeros(numscan, 1); %creates an array of size numscan*1 (20*1) filled with all zeros
    var = 80; %variance is 80%
    dampFactor = 0.00000001; %it is the damping factor
    
    %% Code for weighting the particles   
    for i = 1:num %iterating through each particle and processing it
        if particles(i).insideMap() == 0 %means particle is not present inside the map
            particles(i).randomPose(5); %set random positions for those particles
        end
        arrPartScan(:, i) = particles(i).ultraScan(); %sets the arrPartSacn's value to the ultraScan reults of current particle
        for j = 1:numscan %iterating now for each scan turn
            shiftScanVec = circshift(arrPartScan(:, i), j); %circularly shifts the arrPartScan array by j positions
            arrDiff(j,i) = sqrt(sum((shiftScanVec - botScan) .^ 2)); %calculates the Eculidean distance between the scan vectors
            particleWeight(j) = dampFactor + (1 / sqrt(2 * pi * var)) * exp(-((arrDiff(j, i))^2/(2 * var))); %it is basically a Gaussian distribution formula
        end
        arrWeight(i) = max(particleWeight); %setting the maximum weight of particles
        particleAngle = particles(i).getBotAng() + (max(particleWeight) * 2 * pi / numscan); %calculating the best particle angle
        particles(i).setBotAng(mod(particleAngle, 2 * pi)); %assigning best orientation to the particles based on weights
    end
    %normalizing the wieghts/probability distribution
    normWeight = arrWeight ./ sum(arrWeight);
    
    %% Code for resampling the particles
    newParticleLocations = zeros(num, 3);  %creates an array of num*3 filled with zeros; this will have information about the new particle locations after resampling    
    for i = 1:num
        j = find(rand() <= cumsum(normWeight), 1); %finding one random number such that it is less than the cumulative sum of the normalized particle weight
        newParticleLocations(i, 1:2) = particles(j).getBotPos(); %setting x and y positiong for the particle
        newParticleLocations(i, 3) = particles(j).getBotAng(); %setting the heading angle for the particle
    end 
    for i=1:num
        particles(i).setBotPos([newParticleLocations(i,1), newParticleLocations(i,2)]);
        particles(i).setBotAng(newParticleLocations(i,3));
    end
    
    positionsParticles = zeros(num, 2); %array to store the position of particles 
    anglesParticles = zeros(num,1); %array to store the angle of particles
    for i = 1:num
        positionsParticles(i, :) = particles(i).getBotPos();
        anglesParticles(i) = particles(i).getBotAng();
    end
    
    % ****************** extra code for debug mode
    %estimating the mean for the demo bot
    botDemoMean = BotSim(map);
    botDemoMean.setScanConfig(botDemoMean.generateScanConfig(numscan));
    botDemoMean.setBotPos(mean(positionsParticles));
    botDemoMean.setBotAng(mean(anglesParticles));
    
    %estimating the mode for the demo bot
    botDemoMode = BotSim(map);
    botDemoMode.setScanConfig(botDemoMode.generateScanConfig(numscan));
    botDemoMode.setBotPos(mode(positionsParticles));
    botDemoMode.setBotAng(mean(anglesParticles));
    
    if botSim.debug()
        figure(1)
        hold off; %the drawing is cleard when the hold off function is called
        botSim.drawMap(); %drawing is enabled
        botSim.drawBot(25,'g'); %drawing robot
        for i =1:num
            particles(i).drawBot(5); %drawing the particles
        end
        botDemoMean.drawBot(25, 'r'); %drawing the mean for the demo bot
        botDemoMode.drawBot(25, 'b'); %drawing the mode for the demo bot
        drawnow;
    end 
    % ***************** extra code ends
    
    %% Code for checking the convergence   
	convergenceThreshold = 2; %convergence threshold
    stndev = std(positionsParticles); %setting the standard deviation
    %note that the particle filter will be considered to converge if the standard deviation falls below the convergence threshold
    if stndev < convergenceThreshold
        break; %after the particle filter has converged, we break out of the while loop
    end

    %% Code to respawn the particles	
    respawnRate = 0.01;
    for i=1:respawnRate * num
        particles(randi(num)).randomPose(5);
    end
    
    %% Code to decide the next move for the particles
    if rand() < 0.8 %most of the time move in the maximum direction
        [maxDist, maxIndex] = max(botScan); %set maximum possible distance
        turn = (maxIndex - 1) * 2 * pi / numscan; %orientate towards the max distance
        move = maxDist * 0.8 * rand(); %move a random amount of the max distance, but never the entire distance
    else %somtimes move in a random direction
        index = randi(numscan); %random integer generation
        turn = (index - 1) * 2 * pi / numscan;
        move = botScan(index)*0.8;
    end
        
    botSim.turn(turn); %this command turnns the actual robot  
    botSim.move(move); %this command moves the actual robot
    for i =1:num %processing for every particle
        particles(i).turn(turn); %turning the particles like the actual robot
        particles(i).move(move); %moving the particles like the actual robot
    end    
    
    %% Code to draw the actual robot
    if botSim.debug()
        hold off; %the drawing is cleard when the hold off function is called
        botSim.drawMap(); %drawing is enabled
        botSim.drawBot(25,'g'); %drawing robot
        for i =1:num
            particles(i).drawBot(5); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end

%% Assigning values to botDemo and checking for better metrics
botScan = botSim.ultraScan();
meanDiff= zeros(360, 1);
modeDiff= zeros(360, 1);
for i=1:360 %checking mean and mode for each orientation
    botDemoMeanScan = botDemoMean.ultraScan();
    botDemoModeScan = botDemoMode.ultraScan();
    meanDiff(i) = norm(botDemoMeanScan - botScan);
    modeDiff(i) = norm(botDemoModeScan - botScan);
    botDemoMean.setBotAng(i * pi / 180);
    botDemoMode.setBotAng(i * pi / 180);
end

[minDiffMean, minPosMean] = min(meanDiff);
botDemoMean.setBotAng(minPosMean * pi / 180); 

[minDiffMode, minPosMode] = min(modeDiff);
botDemoMode.setBotAng(minPosMode * pi / 180);

if minDiffMean < minDiffMode %pick best from mean or mode estimates
    botDemo = botDemoMean;
else
    botDemo = botDemoMode;
end