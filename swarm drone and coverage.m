% Define the UAV scenario
scene = uavScenario(ReferenceLocation=[40.707088 -74.012146 0], UpdateRate=5);

% Define terrain limits and add terrain mesh
xTerrainLimits = [-200 200];
yTerrainLimits = [-200 200];
terrainColor = [0.6 0.6 0.6];
addMesh(scene, "terrain", {"gmted2010", xTerrainLimits, yTerrainLimits}, terrainColor);

% Define building limits and add building mesh as obstacles
xBuildingLimits = [-150 150];
yBuildingLimits = [-150 150];
buildingColor = [0.6431 0.8706 0.6275];
addMesh(scene, "buildings", {"manhattan.osm", xBuildingLimits, yBuildingLimits, "auto"}, buildingColor);

% Define obstacle locations (buildings as obstacles)
obstacles = [
    40.707300, -74.011500, 0, 50, 50, 150; % Obstacle 1: [x, y, z, width, depth, height]
    40.706700, -74.012500, 0, 30, 30, 100; % Obstacle 2
];

% Define the number of clusters (equal to the number of drones)
numDrones = 3;
droneColors = [1 0 0; 0 1 0; 0 0 1]; % Different colors for each drone

% Define starting and destination points for the drones
startingPoint = [40.707088, -74.012146, 50]; % Starting point (with an altitude of 50 units)
destinationPoint = [40.706500, -74.013000, 50]; % Destination point (with an altitude of 50 units)

% Generate random points within the terrain limits to serve as area points for clustering
numPoints = 100; % Number of random points
areaPoints = [xTerrainLimits(1) + (xTerrainLimits(2) - xTerrainLimits(1)) * rand(numPoints, 1), ...
              yTerrainLimits(1) + (yTerrainLimits(2) - yTerrainLimits(1)) * rand(numPoints, 1)];

% Apply k-means clustering to the area points
[idx, clusterCenters] = kmeans(areaPoints, numDrones);

% Define the radius of the circular area each drone will cover
coverageRadius = 50; % Radius of coverage circle

% Plot the 3D scene
figure;
show3D(scene);
hold on;

% Plot obstacles
for i = 1:size(obstacles, 1)
    plotcube(obstacles(i, 4:6), obstacles(i, 1:3) - [obstacles(i, 4)/2, obstacles(i, 5)/2, 0], .8, [0.5 0.5 0.5]);
end

% Plot the starting point and destination point
plot3(startingPoint(1), startingPoint(2), startingPoint(3), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'Starting Point');
plot3(destinationPoint(1), destinationPoint(2), destinationPoint(3), 'kd', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'Destination Point');

% Create UAV platforms and assign them to the cluster centers
for i = 1:numDrones
    % Create UAV platform
    plat(i) = uavPlatform("UAV" + string(i), scene);
    
    % Update platform mesh (quadrotor model) with different colors
    updateMesh(plat(i), "quadrotor", {3}, droneColors(i,:), eul2tform([0 0 pi]));
    
    % Create transformation matrix for the starting position
    tformStart = trvec2tform(startingPoint); % Convert starting point to a transformation matrix
    move(plat(i), reshape(tformStart', 1, [])); % Move the UAV to the starting point
    
    % Create transformation matrix for the cluster center
    clusterCenter = [clusterCenters(i, :), 100]; % Assuming an altitude of 100 units
    
    % Find a 3D path avoiding obstacles
    pathToCluster = findPath3D(startingPoint, clusterCenter, obstacles);
    pathToDestination = findPath3D(clusterCenter, destinationPoint, obstacles);
    
    % Plot the path to the cluster center
    plot3(pathToCluster(:,1), pathToCluster(:,2), pathToCluster(:,3), 'Color', droneColors(i,:), 'LineWidth', 2, 'DisplayName', ['Drone ' num2str(i) ' Path to Cluster']);
    
    % Plot the circular coverage area for each drone at the cluster center
    theta = linspace(0, 2*pi, 100); % Angle for circular coverage
    xCircle = clusterCenters(i,1) + coverageRadius * cos(theta);
    yCircle = clusterCenters(i,2) + coverageRadius * sin(theta);
    plot3(xCircle, yCircle, 100*ones(size(xCircle)), '-', 'Color', droneColors(i,:), 'LineWidth', 2, 'DisplayName', ['Drone ' num2str(i) ' Coverage Area']);
    
    % Plot the path to the destination point
    plot3(pathToDestination(:,1), pathToDestination(:,2), pathToDestination(:,3), 'Color', droneColors(i,:), 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', ['Drone ' num2str(i) ' Path to Destination']);
end

% Show the clusters with different colors
gscatter(areaPoints(:,1), areaPoints(:,2), idx, droneColors);

% Add legend to the plot
legend show;

% Adjust view to 3D perspective
view(3);
hold off;

% Function to plot a cube as an obstacle
function plotcube(dim, origin, alpha, color)
    % PLOTCUBE - Display a 3D-cube in the current axes
    %
    %   PLOTCUBE(DIM, ORIGIN, ALPHA, COLOR) displays a 3D-cube in the current
    %   axes with the dimension DIM and the origin ORIGIN.
    %   DIM is a 3-element vector that defines the dimension of the cube.
    %   ORIGIN is a 3-element vector that defines the starting point of the cube.
    %   ALPHA defines the transparency of the cube face (0 = transparent, 1 = opaque).
    %   COLOR defines the face color of the cube.
    
    X = [0 1 1 0 0 1 1 0]*dim(1) + origin(1);
    Y = [0 0 1 1 0 0 1 1]*dim(2) + origin(2);
    Z = [0 0 0 0 1 1 1 1]*dim(3) + origin(3);
    
    vert = [X(:) Y(:) Z(:)];
    fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    
    patch('Faces', fac, 'Vertices', vert, 'FaceColor', color, 'FaceAlpha', alpha);
end

% Function to find a 3D path avoiding obstacles
function path = findPath3D(startPos, endPos, obstacles)
    % Simplified example of a pathfinding algorithm in 3D space
    % This function generates a straight path from startPos to endPos
    % and adjusts it to avoid obstacles (this is a placeholder for a
    % more sophisticated pathfinding algorithm).
    
    path = [startPos; endPos]; % Start with a straight path
    
    % Iterate over obstacles and adjust the path to avoid them
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        obstacleCenter = obstacle(1:3);
        obstacleSize = obstacle(4:6) / 2; % Half-size
        
        % Check if the path intersects with the obstacle
        if any(inpolygon(path(:,1), path(:,2), ...
                [obstacleCenter(1) - obstacleSize(1), obstacleCenter(1) + obstacleSize(1)], ...
                [obstacleCenter(2) - obstacleSize(2), obstacleCenter(2) + obstacleSize(2)]))
            % Adjust path around the obstacle (simple example)
            midPoint = (startPos + endPos) / 2;
            midPoint(3) = midPoint(3) + obstacleSize(3); % Move upwards to avoid
            path = [startPos; midPoint; endPos];
        end
    end
end
