clear
clc
% Set up initial positions of UAVs
pos1 = [-2 -7];
pos2 = [10 8];

% Set up formation control parameters
K = 0.05; % Gain
d = 5; % Desired distance between UAVs

% Set up animation parameters
dt = 0.1; % Time step
T = 10; % Total time
frames = T / dt; % Number of frames

% Set up figure
fig = figure();
axis square;
hold on;
line([pos1(1) pos2(1)], [pos1(2) pos2(2)], 'LineStyle', '--', 'Color', 'k');
plot(pos1(1), pos1(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(pos2(1), pos2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
xlabel('X');
ylabel('Y');
title('Formation Control of Two UAVs');

% Initialize position arrays
pos1_arr = zeros(frames, 2);
pos2_arr = zeros(frames, 2);

% Set initial positions
pos1_arr(1, :) = pos1;
pos2_arr(1, :) = pos2;

% Animate the movement of the UAVs
for i = 2:frames
    % Calculate the error vector
    e = pos1 - pos2 + [d 0];
    
    % Update the positions of the UAVs
    pos1 = pos1 - K * e;
    pos2 = pos2 + K * e;
    
    % Save the positions to the arrays
    pos1_arr(i, :) = pos1;
    pos2_arr(i, :) = pos2;
    
    % Update the figure
    clf;
    axis square;
    hold on;
    line([pos1(1) pos2(1)], [pos1(2) pos2(2)], 'LineStyle', '--', 'Color', 'k');
    plot(pos1_arr(1:i, 1), pos1_arr(1:i, 2), 'r-', 'LineWidth', 2);
    plot(pos1(1), pos1(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(pos2_arr(1:i, 1), pos2_arr(1:i, 2), 'b-', 'LineWidth', 2);
    plot(pos2(1), pos2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    xlabel('X');
    ylabel('Y');
    title('Formation Control of Two UAVs');
    
    % Pause for animation
    pause(dt);
end
