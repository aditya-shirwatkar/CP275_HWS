clc
clear
%% Discrete time model of a quadcopter
A_uav = [1       0       0   0   0   0   0.1     0       0    0       0       0;
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
B_uav = [0      -0.0726  0       0.0726;
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

Ad = blkdiag(A_uav, A_uav);
Bd = blkdiag(B_uav, B_uav);

[nx, nu] = size(Bd);

% Constraints
u0 = 10.5916;
u_uav_min = [9.6; 9.6; 9.6; 9.6] - u0;
u_uav_max = [13; 13; 13; 13] - u0;
x_uav_min = [-pi/6; -pi/6; -Inf; -Inf; -Inf; -1; -Inf(6,1)];
x_uav_max = [ pi/6;  pi/6;  Inf;  Inf;  Inf; Inf; Inf(6,1)];

umin = [u_uav_min; u_uav_min];
umax = [u_uav_max; u_uav_max];
xmin = [x_uav_min; x_uav_min];
xmax = [x_uav_max; u_uav_max];

% Objective function
Q_uav = diag([0 0 10 10 10 10 0 0 0 5 5 5]);
Q = blkdiag(Q_uav, Q_uav);
QN = Q;
R_uav = 0.1*eye(4);
R = blkdiag(R_uav, R_uav);

% Initial and reference states
% [roll, pitch, yaw, x, y, z, roll_dot, pitch_dot, yaw_dot, vx, vy, vz]
x0_uav1 = [0; 0; 0; -0.5; -0.5; 0; 0; 0; 0; 0; 0; 0];
x0_uav2 = [0; 0; 0; 0.5; 0.5; 0; 0; 0; 0; 0; 0; 0];

x0 = [x0_uav1; x0_uav2];

xr_uav1 = [0; 0; pi/3; -1; 0; 2; 0; 0; 0; 0; 0; 0];
xr_uav2 = [0; 0; pi/3; 1; 0; 2; 0; 0; 0; 0; 0; 0];

xr = [xr_uav1; xr_uav2];

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
X = zeros(nx, nsim+1);
X(:, 1) = x0;
for i = 1 : nsim
    U = controller{X(:,i)};
    X(:, i+1) = Ad*X(:, i) + Bd*U(:,1);
end

% Extract position and orientation vectors
euler1 = X(1:3, :);
pos1 = X(4:6, :);

euler2 = X(13:15, :);
pos2 = X(16:18, :);

% Compute rotation matrices
Rz = @(theta) [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
Rx = @(theta) [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
Ry = @(theta) [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];

L = 0.25;

%% Preallocate animation objects
% Initialize video
myVideo = VideoWriter('multi_3D_uav_qp'); %open video file
myVideo.FrameRate = 24;  %can adjust this, 5 - 10 works well for me
open(myVideo)

figure();
axis equal;
% Animate quadcopter flying in circle
for i = 1:nsim
    % Compute rotation matrix
    R1 = Rz(euler1(3, i))*Ry(euler1(2, i))*Rx(euler1(1, i));
    % Define vertices of quadcopter
    cuav1_1 = 3*R1*[-L; 0; 0];
    cuav1_2 = 3*R1*[L; 0; 0];
    cuav1_3 = 3*R1*[0; L; 0];
    cuav1_4 = 3*R1*[0; -L; 0];
    cuav1_5 = 3*R1*[0; 0; 0];
    cuav1_6 = 3*R1*[0; 0; -L];

    R2 = Rz(euler2(3, i))*Ry(euler2(2, i))*Rx(euler2(1, i));
    % Define vertices of quadcopter
    cuav2_1 = 3*R2*[-L; 0; 0];
    cuav2_2 = 3*R2*[L; 0; 0];
    cuav2_3 = 3*R2*[0; L; 0];
    cuav2_4 = 3*R2*[0; -L; 0];
    cuav2_5 = 3*R2*[0; 0; 0];
    cuav2_6 = 3*R2*[0; 0; -L];

    % Plot quadcopter
    clf;
    axis equal;
    hold on;

    % UAV 1
    plot3(pos1(1, 1:i), pos1(2, 1:i), pos1(3, 1:i), 'b', 'LineWidth', 1);
    plot3(pos1(1, i)+[cuav1_1(1), cuav1_2(1)], pos1(2, i)+[cuav1_1(2), cuav1_2(2)], pos1(3, i)+[cuav1_1(3), cuav1_2(3)], 'r', 'LineWidth', 2);
    plot3(pos1(1, i)+[cuav1_3(1), cuav1_4(1)], pos1(2, i)+[cuav1_3(2), cuav1_4(2)], pos1(3, i)+[cuav1_3(3), cuav1_4(3)], 'g', 'LineWidth', 2);
    plot3(pos1(1, i)+[cuav1_5(1), cuav1_6(1)], pos1(2, i)+[cuav1_5(2), cuav1_6(2)], pos1(3, i)+[cuav1_5(3), cuav1_6(3)], 'b', 'LineWidth', 2);

    plot3(pos2(1, 1:i), pos2(2, 1:i), pos2(3, 1:i), 'b', 'LineWidth', 1);
    plot3(pos2(1, i)+[cuav2_1(1), cuav2_2(1)], pos2(2, i)+[cuav2_1(2), cuav2_2(2)], pos2(3, i)+[cuav2_1(3), cuav2_2(3)], 'r', 'LineWidth', 2);
    plot3(pos2(1, i)+[cuav2_3(1), cuav2_4(1)], pos2(2, i)+[cuav2_3(2), cuav2_4(2)], pos2(3, i)+[cuav2_3(3), cuav2_4(3)], 'g', 'LineWidth', 2);
    plot3(pos2(1, i)+[cuav2_5(1), cuav2_6(1)], pos2(2, i)+[cuav2_5(2), cuav2_6(2)], pos2(3, i)+[cuav2_5(3), cuav2_6(3)], 'b', 'LineWidth', 2);

    xlim([-2, 2]);
    ylim([-2, 2]);
    zlim([-0.5, 2]);
    view(3);

    % drawnow;
    plot3(x0_uav1(4), x0_uav1(5), x0_uav1(6), 'r*');
    plot3(xr_uav1(4), xr_uav1(5), xr_uav1(6), 'g*');

    plot3(x0_uav2(4), x0_uav2(5), x0_uav2(6), 'r*');
    plot3(xr_uav2(4), xr_uav2(5), xr_uav2(6), 'g*');

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    pause(0.01);
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)
