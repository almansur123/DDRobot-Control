clc;
clear;
close all;

% Define map and parameters
mapSize = 10; % 10x10 grid
numGoals = 3; % Number of goal points
robotRadius = 0.2; % Robot size (radius)
obstacleDensity = 0.15; % Fraction of the area occupied by obstacles
robotSpeed = 0.2; % Robot speed
robotAngle = 0; % Initial robot orientation

% Initialize the map
map = zeros(mapSize, mapSize);

% Randomly generate obstacles, excluding goal positions
obstacles = rand(mapSize, mapSize) < obstacleDensity;

% Generate random goal points that are not occupied by obstacles
goals = generateRandomGoals(numGoals, mapSize, obstacles);

% Initialize the robot's starting position and odometry
robotPos = [1, 1];
odometryPos = robotPos; % Odometry position

% Create the figure for visualization
figure;
hold on;
axis([0 mapSize 0 mapSize]);
grid on;
set(gca, 'xtick', 0:mapSize);
set(gca, 'ytick', 0:mapSize);

% Plot goals and obstacles
plotGoalsAndObstacles(goals, obstacles);

% Plot robot's initial position
robotPlot = plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

% Initialize the path line
pathLine = plot(robotPos(1), robotPos(2), 'b-', 'LineWidth', 1.5);
pathPoints = robotPos; % Initialize path points with the starting position

% Main loop to navigate through goals
for goalIdx = 1:numGoals
    % Move the robot from the current position to the next goal
    [robotPos, odometryPos, robotAngle, pathPoints] = navigateToGoal(robotPos, goals(goalIdx, :), obstacles, robotSpeed, mapSize, robotPlot, robotRadius, goalIdx, robotAngle, odometryPos, pathPoints, pathLine);
end

% End of simulation
disp('Simulation complete!');

%% Supporting Functions

% Function to generate random goal points avoiding obstacles
function goals = generateRandomGoals(numGoals, mapSize, obstacles)
    goals = zeros(numGoals, 2); % Initialize goals array
    for i = 1:numGoals
        while true
            % Generate a random position within the map
            randX = randi(mapSize);
            randY = randi(mapSize);
            
            % Check if the position is occupied by an obstacle
            if obstacles(randX, randY) == 0
                goals(i, :) = [randX, randY]; % Assign goal if no obstacle
                break;
            end
        end
    end
end

% Function to plot goals and obstacles
function plotGoalsAndObstacles(goals, obstacles)
    % Plot goals
    for i = 1:size(goals, 1)
        plot(goals(i, 1), goals(i, 2), 'go', 'MarkerSize', 10, 'LineWidth', 3); % Green goals
    end
    
    % Plot obstacles
    [xObst, yObst] = find(obstacles);
    plot(xObst, yObst, 'ro', 'MarkerSize', 6); % Red obstacles
end

% Function to navigate robot to the goal using A* and dynamic motion
function [robotPos, odometryPos, robotAngle, pathPoints] = navigateToGoal(robotPos, goal, obstacles, robotSpeed, mapSize, robotPlot, robotRadius, goalIdx, robotAngle, odometryPos, pathPoints, pathLine)    % Use A* pathfinding to get a path from current position to goal
    path = aStarPathfinding(robotPos, goal, obstacles, mapSize);
    
    % Apply Bézier curve smoothing
    if ~isempty(path)
    smoothPath = bezierSmoothPath(path);

    else
        disp('No path found!');
        return;
    end

    % Kinematics parameters
    maxSpeed = 0.2;        % Maximum movement speed
    maxTurnRate = 0.1;     % Maximum turn rate per step
    turnTolerance = 0.05;  % Allowable angle error before moving forward
    goalThreshold = 0.2;   % Distance threshold to determine goal completion
    
    % Iterate through the path **only once**
    for i = 1:size(smoothPath, 1)
        nextPos = smoothPath(i, :);
        
        while true
            % Compute direction and target angle
            direction = nextPos - robotPos;
            targetAngle = atan2(direction(2), direction(1));
            
            % Compute angle difference
            angleError = targetAngle - robotAngle;
            angleError = mod(angleError + pi, 2 * pi) - pi; % Normalize angle
            
            % Apply gradual turning
            if abs(angleError) > turnTolerance
                turnRate = sign(angleError) * min(maxTurnRate, abs(angleError));
                robotAngle = robotAngle + turnRate;
            else
                % Move forward once aligned
                distance = norm(direction);
                speed = min(maxSpeed, distance);
                robotPos = robotPos + speed * [cos(robotAngle), sin(robotAngle)];
            end
            
            % Ensure robot stays within the map
            robotPos = max(min(robotPos, mapSize), 1);
            
            % **Append new robot position to pathPoints to avoid duplication**
            pathPoints = [pathPoints; robotPos];
            
            % **Update visualization only once per movement step**
            set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2));
            set(pathLine, 'XData', pathPoints(:, 1), 'YData', pathPoints(:, 2));
            
            drawnow; % Refresh visualization instantly
            pause(0.02); % Adjust speed
            
            % **Stop when the robot reaches the next waypoint**
            if norm(robotPos - nextPos) < 0.1
                break;
            end
        end
    end

    % **FINAL GOAL REACHED CHECK: Stop unnecessary returns**
    if norm(robotPos - goal) < goalThreshold
        disp(['Goal ', num2str(goalIdx), ' reached!']);
        return;
    end

    % Iterate through the path
    for i = 1:size(smoothPath, 1) - 1
    % Get the current and next target position
    currentPos = smoothPath(i, :);
    nextPos = smoothPath(i + 1, :);
    
    % Compute direction and step size
    direction = nextPos - currentPos;
    stepSize = 0.05; % Adjust step size for smoother movement
    numSteps = ceil(norm(direction) / stepSize); % Compute required steps
    
    % Move gradually from currentPos to nextPos
    for step = 1:numSteps
        % Interpolate between the points
        robotPos = currentPos + (direction * (step / numSteps));
        
        % Update robot plot
        set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2));
        
        % Append new point to path
        pathPoints = [pathPoints; robotPos];
        set(pathLine, 'XData', pathPoints(:, 1), 'YData', pathPoints(:, 2));
        
        % Update robot plot
        set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2));

        % Update path line
        set(pathLine, 'XData', pathPoints(:, 1), 'YData', pathPoints(:, 2));
        drawnow; % Refresh visualization instantly
        pause(0.02); % Small pause to slow down execution
    end
end
        
        % Implement obstacle avoidance
        robotPos = reactiveAvoidance(robotPos, obstacles, robotSpeed, mapSize, robotPlot, robotRadius);

        % Smooth movement towards the goal
        robotPos = smoothMove(robotPos, goal, robotSpeed);

        % Update robot's angle
        robotAngle = updateRobotAngle(robotPos, goal, robotAngle);
        
        % Move robot based on angle
        robotPos = moveRobot(robotPos, robotAngle, robotSpeed);

        % Update odometry position
        odometryPos = updateOdometry(odometryPos, robotPos, robotAngle, robotSpeed);

        % Update the robot position in the plot
        set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2));
        
        % Update the path points
        pathPoints = [pathPoints; robotPos];
        
        % Update the path line
        set(pathLine, 'XData', pathPoints(:, 1), 'YData', pathPoints(:, 2));
        
        % Update robot plot
        set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2));

        % Update path line
        set(pathLine, 'XData', pathPoints(:, 1), 'YData', pathPoints(:, 2));
        drawnow; % Refresh visualization instantly
        pause(0.02); % Small pause to slow down execution
end

% Function for A* pathfinding using a priority queue
function path = aStarPathfinding(startPos, goal, obstacles, mapSize)
    % Initialize structures for A*
    openSet = containers.Map; % Set of open nodes (positions)
    openSet(num2str(startPos)) = heuristic(startPos, goal); % Start node with initial f-score
    cameFrom = containers.Map; % Map to store the best path to each node
    gScore = containers.Map('KeyType', 'char', 'ValueType', 'double'); % Cost from start to node
    gScore(num2str(startPos)) = 0; % Start node has 0 g-score

    while ~isempty(openSet)
        % Find the node in openSet with the lowest fScore
        [currentPos, ~] = minKey(openSet);
        openSet.remove(currentPos); % Remove the current node from openSet
        currentPos = str2num(currentPos);

        % If we have reached the goal, reconstruct the path
        if isequal(currentPos, goal)
            path = reconstructPath(cameFrom, currentPos);
            return;
        end

        % Generate neighbors
        neighbors = getNeighbors(currentPos, mapSize);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            
            % Ensure the neighbor is within the valid grid bounds
            if neighbor(1) < 1 || neighbor(1) > mapSize || neighbor(2) < 1 || neighbor(2) > mapSize
                continue; % Skip if the neighbor is out of bounds
            end

            % Ensure the neighbor coordinates are integers
            neighbor = round(neighbor);

            % Check if the neighbor is not an obstacle
            if obstacles(neighbor(1), neighbor(2)) == 1
                continue; % Skip if the neighbor is an obstacle
            end

            % Calculate gScore for neighbor
            tentativeGScore = gScore(num2str(currentPos)) + 1; % Assuming uniform cost for all moves
            if ~isKey(gScore, num2str(neighbor)) || tentativeGScore < gScore(num2str(neighbor))
                cameFrom(num2str(neighbor)) = currentPos;
                gScore(num2str(neighbor)) = tentativeGScore;
                openSet(num2str(neighbor)) = gScore(num2str(neighbor)) + heuristic(neighbor, goal);
            end
        end
    end
    path = []; % Return empty if no path found
end

% Heuristic function for A* (Euclidean distance)
function h = heuristic(pos, goal)
    h = sqrt((pos(1) - goal(1))^2 + (pos(2) - goal(2))^2); % Euclidean distance
end

function neighbors = getNeighbors(pos, mapSize)
    neighbors = [
        pos + [1, 0];  % Right
        pos + [-1, 0]; % Left
        pos + [0, 1];  % Up
        pos + [0, -1]; % Down
        pos + [1, 1];  % Diagonal top-right
        pos + [-1, -1]; % Diagonal bottom-left
        pos + [1, -1]; % Diagonal bottom-right
        pos + [-1, 1]; % Diagonal top-left
    ];

    % Filter neighbors that are out of bounds
    neighbors = neighbors(all(neighbors >= 1 & neighbors <= mapSize, 2), :);
end

% Reconstruct the path from start to goal
function path = reconstructPath(cameFrom, currentPos)
    path = currentPos;
    while isKey(cameFrom, num2str(currentPos))
        currentPos = cameFrom(num2str(currentPos));
        path = [currentPos; path];
    end
end

function smoothPath = bezierSmoothPath(path)
    t = linspace(0, 1, 100); % Parameter for the curve
    n = size(path, 1) - 1; % Degree of the Bézier curve

    % Compute Bernstein polynomials
    smoothPath = zeros(length(t), 2);
    for i = 0:n
        B = nchoosek(n, i) .* (t .^ i) .* ((1 - t) .^ (n - i));
        smoothPath = smoothPath + B' * path(i + 1, :);
    end
end

% Function to find the minimum key (position) in a Map
function [key, val] = minKey(openSet)
    keys = openSet.keys;
    [~, idx] = min(cell2mat(values(openSet)));
    key = keys{idx};
    val = openSet(key);
end

% Function for Reactive Obstacle Avoidance with smarter behavior
function robotPos = reactiveAvoidance(robotPos, obstacles, robotSpeed, mapSize, robotPlot, robotRadius)
    % Define avoidance parameters
    proximityRadius = robotRadius + 1; % Distance for obstacle avoidance
    [xObst, yObst] = find(obstacles); % Find all obstacles
    
    for i = 1:length(xObst)
        dist = sqrt((xObst(i) - robotPos(1))^2 + (yObst(i) - robotPos(2))^2);
        
        if dist < proximityRadius
            % Calculate angle to obstacle and determine avoidance direction
            angleToObstacle = atan2(yObst(i) - robotPos(2), xObst(i) - robotPos(1));
            avoidanceAngle = angleToObstacle + pi / 2; % Turn 90 degrees for avoidance
            robotPos = robotPos + [cos(avoidanceAngle) * 0.5, sin(avoidanceAngle) * 0.5]; % Move away
            break; % Exit after first avoidance
        end
    end
    
    set(robotPlot, 'XData', robotPos(1), 'YData', robotPos(2)); % Update plot
end

% Function to move robot smoothly toward goal
function robotPos = smoothMove(robotPos, goal, robotSpeed)
    direction = goal - robotPos;
    distance = norm(direction);
    
    if distance < robotSpeed
        robotPos = goal; % Goal reached
    else
        robotPos = robotPos + robotSpeed * direction / distance; % Move towards goal
    end
end

% Function to update robot's angle based on position relative to goal
function robotAngle = updateRobotAngle(robotPos, goal, robotAngle)
    direction = goal - robotPos;
    robotAngle = atan2(direction(2), direction(1)); % Update angle to face the goal
end

% Function to move robot based on its angle
function robotPos = moveRobot(robotPos, robotAngle, robotSpeed)
    robotPos = robotPos + robotSpeed * [cos(robotAngle), sin(robotAngle)];
end

% Function to update odometry position
function odometryPos = updateOdometry(odometryPos, robotPos, robotAngle, robotSpeed)
    odometryPos = odometryPos + robotSpeed * [cos(robotAngle), sin(robotAngle)];
end
