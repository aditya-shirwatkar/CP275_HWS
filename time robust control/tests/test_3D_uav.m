clc
clear
%% Discrete time model of a quadcopter
Ad = [1       0       0   0   0   0   0.1     0       0    0       0       0;
      0       1       0   0   0   0   0       0.1     0    0       0       0;
      0       0       1   0   0   0   0       0       0.1  0       0       0;
      0.0488  0       0   1   0   0   0.0016  0       0    0.0992  0       0;
      0      -0.0488  0   0   1   0   0      -0.0016  0    0       0.0992  0;
      0       0       0   0   0   1   0       0       0    0       0       0.0992;
      0       0       0   0   0   0   1       0       0    0       0       0;
      0       0       0   0   0   0   0       1       0    0       0       0;
      0       0       0   0   0   0   0       0       1    0       0       0;
      0.9734  0       0   0   0   0   0.0488  0       0    0.9846  0       0;
      0      -0.9734  0   0   0   0   0      -0.0488  0    0       0.9846  0;
      0       0       0   0   0   0   0       0       0    0       0       0.9846];
Bd = [0      -0.0726  0       0.0726;
     -0.0726  0       0.0726  0;
     -0.0152  0.0152 -0.0152  0.0152;
      0      -0.0006 -0.0000  0.0006;
      0.0006  0      -0.0006  0;
      0.0106  0.0106  0.0106  0.0106;
      0      -1.4512  0       1.4512;
     -1.4512  0       1.4512  0;
     -0.3049  0.3049 -0.3049  0.3049;
      0      -0.0236  0       0.0236;
      0.0236  0      -0.0236  0;
      0.2107  0.2107  0.2107  0.2107];
[nx, nu] = size(Bd);

% Constraints
u0 = 10.5916;
umin = [9.6; 9.6; 9.6; 9.6] - u0;
umax = [13; 13; 13; 13] - u0;
xmin = [-pi/6; -pi/6; -Inf; -Inf; -Inf; -1; -Inf(6,1)];
xmax = [ pi/6;  pi/6;  Inf;  Inf;  Inf; Inf; Inf(6,1)];

% Objective function
Q = diag([0 0 10 10 10 10 0 0 0 5 5 5]);
QN = Q;
R = 0.1*eye(4);

% Initial and reference states
x0 = zeros(12,1);
% [roll, pitch, yaw, x, y, z, roll_dot, pitch_dot, yaw_dot, vx, vy, vz]
xr = [0; 0; pi/3; 1; 1; 2; 0; 0; 0; 0; 0; 0];

% Prediction horizon
N = 10;

%% Define shooting problem
u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
x0_sdp = sdpvar(nx, 1);
objective = 0;
constraints = [];
x = x0_sdp;
for k = 1 : N
    objective = objective + (x-xr)'*Q*(x-xr) + u{k}'*R*u{k};
    constraints = [constraints, umin <= u{k}<= umax];
    x = Ad*x + Bd*u{k};
end
objective = objective + (x-xr)'*QN*(x-xr);
options = sdpsettings('solver', 'mosek');
controller = optimizer(constraints, objective, options, x0_sdp, [u{:}]);

%% Define collocation problem
% u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
% x = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
% constraints = [xmin <= x{1} <= xmax];
% objective = 0;
% for k = 1 : N
%     objective = objective + (x{k}-xr)'*Q*(x{k}-xr) + u{k}'*R*u{k};
%     constraints = [constraints, x{k+1} == Ad*x{k} + Bd*u{k}];
%     constraints = [constraints, umin <= u{k}<= umax, xmin <= x{k+1} <= xmax];
% end
% objective = objective + (x{N+1}-xr)'*QN*(x{N+1}-xr);
% options = sdpsettings('solver', 'mosek');
% controller = optimizer(constraints, objective, options, x{1}, [u{:}]);


%% Simulate in closed loop
nsim = 50;

% Preallocate state and input arrays
X = zeros(12, nsim+1);
X(:, 1) = x0;

for i = 1 : nsim
    U = controller{X(:,i)};
    X(:, i+1) = Ad*X(:, i) + Bd*U(:,1);
end

% Extract position and orientation vectors
euler = X(1:3, :);
pos = X(4:6, :);

% Compute rotation matrices
Rz = @(theta) [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
Rx = @(theta) [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
Ry = @(theta) [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];

L = 0.25;

% Preallocate animation objects
figure();
axis equal;
% Animate quadcopter flying in circle
for i = 1:nsim
    % Compute rotation matrix
    R = Rz(euler(3, i))*Ry(euler(2, i))*Rx(euler(1, i));
    % Define vertices of quadcopter
    c1 = 3*R*[-L; 0; 0];
    c2 = 3*R*[L; 0; 0];
    c3 = 3*R*[0; L; 0];
    c4 = 3*R*[0; -L; 0];
    c5 = 3*R*[0; 0; 0];
    c6 = 3*R*[0; 0; -L];

    % Plot quadcopter
    clf;
    axis equal;
    hold on;
    plot3(pos(1, 1:i), pos(2, 1:i), pos(3, 1:i), 'b', 'LineWidth', 1);
    plot3(pos(1, i)+[c1(1), c2(1)], pos(2, i)+[c1(2), c2(2)], pos(3, i)+[c1(3), c2(3)], 'r', 'LineWidth', 2);
    plot3(pos(1, i)+[c3(1), c4(1)], pos(2, i)+[c3(2), c4(2)], pos(3, i)+[c3(3), c4(3)], 'g', 'LineWidth', 2);
    plot3(pos(1, i)+[c5(1), c6(1)], pos(2, i)+[c5(2), c6(2)], pos(3, i)+[c5(3), c6(3)], 'b', 'LineWidth', 2);
    xlim([-2, 2]);
    ylim([-2, 2]);
    zlim([-0.5, 2]);
    view(3);
    % drawnow;
    plot3(x0(4), x0(5), x0(6), 'r*');
    plot3(xr(4), xr(5), xr(6), 'g*');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    pause(0.01);
end