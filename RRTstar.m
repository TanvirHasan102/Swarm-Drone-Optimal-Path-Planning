% Load a 3-D occupancy map of a city block into the workspace
mapData = load("dMapCityBlock.mat");
omap = mapData.omap;
omap.FreeThreshold = 0.5;

% Inflate the occupancy map to add a buffer zone for safe operation around the obstacles
inflate(omap, 1)

% Create an SE(3) state space object with bounds for state variables
ss = stateSpaceSE3([0 220; 0 220; 0 100; -inf inf; -inf inf; -inf inf; -inf inf]);

% Create a 3-D occupancy map state validator using the created state space
% Assign the occupancy map to the state validator object
% Specify the sampling distance interval
sv = validatorOccupancyMap3D(ss, ...
    'Map', omap, ...
    'ValidationDistance', 0.1);

% Create an RRT* path planner with increased maximum connection distance and reduced maximum number of iterations
% Specify a custom goal function that determines that a path reaches the goal if the Euclidean distance to the target is below a threshold of 1 meter
planner = plannerRRTStar(ss, sv, ...
    'MaxConnectionDistance', 50, ...
    'MaxIterations', 1000, ...
    'GoalReachedFcn', @(~, s, g) (norm(s(1:3) - g(1:3)) < 1), ...
    'GoalBias', 0.1);

% Define start poses for the swarm of drones
numDrones = 5;
startPoses = [40 180 25 0.7 0.2 0 0.1; ...
              45 180 25 0.7 0.2 0 0.1; ...
              35 180 25 0.7 0.2 0 0.1; ...
              40 185 25 0.7 0.2 0 0.1; ...
              40 175 25 0.7 0.2 0 0.1];

% Define multiple waypoints
waypoints = [150 33 35 0.3 0 0.1 0.6; ...
             155 33 35 0.3 0 0.1 0.6; ...
             145 33 35 0.3 0 0.1 0.6; ...
             150 38 35 0.3 0 0.1 0.6; ...
             150 28 35 0.3 0 0.1 0.6; ...
             160 40 35 0.3 0 0.1 0.6; ...
             130 50 35 0.3 0 0.1 0.6; ...
             140 60 35 0.3 0 0.1 0.6; ...
             120 70 35 0.3 0 0.1 0.6; ...
             110 80 35 0.3 0 0.1 0.6];

% Define the common landing point for all drones
commonLandingPoint = [100 50 5 0 0 0 0]; % Common landing point

% Initialize cell array to hold waypoints for each drone
droneWaypoints = cell(numDrones, 1);
for i = 1:numDrones
    droneWaypoints{i} = [];
end

% Assign waypoints to drones ensuring no waypoint is visited more than once by any drone
for i = 1:size(waypoints, 1)
    droneWaypoints{mod(i-1, numDrones) + 1} = [droneWaypoints{mod(i-1, numDrones) + 1}; waypoints(i, :)];
end

% Add the common landing point to each drone's waypoints
for i = 1:numDrones
    droneWaypoints{i} = [droneWaypoints{i}; commonLandingPoint];
end

% Configure the random number generator for repeatable result
rng(1, "twister");

figure;
show(omap)
axis equal
view([-10 55])
hold on
colors = lines(numDrones);

% Initialize total cost
totalCost = 0;

% Plan paths for each drone to visit its assigned waypoints
paths = cell(numDrones, 1);
for i = 1:numDrones
    currentPose = startPoses(i, :);
    dronePath = currentPose;  % Initialize the drone path with the start pose
    for j = 1:size(droneWaypoints{i}, 1)
        goalPose = droneWaypoints{i}(j, :);
        [pthObj, solnInfo] = plan(planner, currentPose, goalPose);
        
        % Calculate the cost of the path segment
        segmentCost = sum(sqrt(sum(diff(pthObj.States(:, 1:3)).^2, 2)));
        totalCost = totalCost + segmentCost;
        
        % Append the path segment to the drone path
        dronePath = [dronePath; pthObj.States];
        
        % Update current pose to the last goal pose
        currentPose = goalPose;
    end
    paths{i} = dronePath;  % Store the full path for each drone
end

% Find the maximum length of the paths
maxPathLength = max(cellfun(@(p) size(p, 1), paths));

% Interpolate all paths to have the same number of steps
numSteps = 100; % Number of steps in the animation
interpolatedPaths = cell(numDrones, 1);
for i = 1:numDrones
    path = paths{i};
    interpolatedPath = [];
    for j = 1:size(path, 1)-1
        for k = linspace(0, 1, numSteps)
            interpolatedPath = [interpolatedPath; path(j, 1:3) * (1 - k) + path(j+1, 1:3) * k];
        end
    end
    interpolatedPaths{i} = interpolatedPath;
end

% Initialize drone markers
droneMarkers = gobjects(numDrones, 1);
for i = 1:numDrones
    droneMarkers(i) = scatter3(startPoses(i, 1), startPoses(i, 2), startPoses(i, 3), 50, colors(i, :), 'filled');
end

% Animate the drones flying simultaneously
maxSteps = max(cellfun(@(p) size(p, 1), interpolatedPaths));
for step = 1:maxSteps
    for i = 1:numDrones
        if step <= size(interpolatedPaths{i}, 1)
            % Update the drone's position
            pos = interpolatedPaths{i}(step, 1:3);
            set(droneMarkers(i), 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
        end
    end
    pause(0.05); % Adjust the pause value to control the speed of the animation
end

% Plot the final paths
for i = 1:numDrones
    plot3(interpolatedPaths{i}(:, 1), interpolatedPaths{i}(:, 2), interpolatedPaths{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 2)
end

xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
title('3D RRT* Path Planning for Swarm Drones with Common Landing Point')
hold off

% Display the total cost
disp(['Total Cost of Path Planning: ', num2str(totalCost)]);
