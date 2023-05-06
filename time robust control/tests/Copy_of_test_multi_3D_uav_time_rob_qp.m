clc
clear

%% Define params
params.Nsteps = 50;
params.N = params.Nsteps-1;

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
u_uav_max = [15; 15; 15; 15] - u0;
x_uav_min = [-pi/6; -pi/6; -pi; -2; -2;  -0; -6.0*pi; -6.0*pi; -2.5*pi; -2; -2; -10];
x_uav_max = [ pi/6;  pi/6;  pi;  2;  2; 2.5;  6.0*pi;  6.0*pi;  2.5*pi;  2;  2;  10];

umin = [u_uav_min; u_uav_min]';
umax = [u_uav_max; u_uav_max]';
xmin = [x_uav_min; x_uav_min]';
xmax = [x_uav_max; x_uav_max]';

% Initial and reference states
% [   1,     2,   3, 4, 5, 6,        7,         8,       9, 10, 11, 12]
% [roll, pitch, yaw, x, y, z, roll_dot, pitch_dot, yaw_dot, vx, vy, vz]
x0_uav1 = [0; 0; 0; -0.5; -0.5; 0; 0; 0; 0; 0; 0; 0];
x0_uav2 = [0; 0; 0; 0.5; 0.5; 0; 0; 0; 0; 0; 0; 0];

x0 = [x0_uav1; x0_uav2]';

xr_uav1 = [0; 0; pi/3; -1; 0; 2; 0; 0; 0; 0; 0; 0];
xr_uav2 = [0; 0; pi/3;  1; 0; 2; 0; 0; 0; 0; 0; 0];

xr = [xr_uav1; xr_uav2];

%%% Define opt problem
u = sdpvar(params.N, nu); % Control inputs (N x nu)

Q_uav = diag([0 0 10 10 10 10 0 0 0 5 5 5]);
Q = blkdiag(Q_uav, Q_uav);
QN = Q;
R_uav = 1e-10*eye(4);
R = blkdiag(R_uav, R_uav);

objective = 0;
%% Dynamics
x = x0;
for k = 1:params.N
    x_k = x(end,:)';
    u_k = u(k,:)';
    objective = objective - u_k' * R * u_k; 
    %wk = [0; 0.01]* normrnd(0,1);
    x_kp1 = Ad*x_k + Bd*u_k;
    x = [x; x_kp1'];
end

%% System Constraints
constraints = [umin.*ones(params.N, nu) <= u <= umax.*ones(params.N, nu)];

% For Linear Velocity
for state_indx = 10:11
    for uav_indx = 1:2
        vel_bds = xmin(:,state_indx+12*(uav_indx-1))*ones(params.Nsteps, 1) <= x(:, state_indx+12*(uav_indx-1)) <= xmax(:,state_indx+12*(uav_indx-1))*ones(params.Nsteps, 1);
        constraints = [constraints; vel_bds];
    end
end

% For Angular Velocity
for state_indx = 7:9
    for uav_indx = 1:2
        ang_vel_bds = xmin(:,state_indx+12*(uav_indx-1))*ones(params.Nsteps, 1) <= x(:, state_indx+12*(uav_indx-1)) <= xmax(:,state_indx+12*(uav_indx-1))*ones(params.Nsteps, 1);
        constraints = [constraints; ang_vel_bds];
    end
end

% Workspace constraints
for state_indx = 4:6
    for uav_indx = 1:2
        workspace_bds = xmin(:,state_indx+12*(uav_indx-1))*ones(params.Nsteps, 1) <= x(:, state_indx+12*(uav_indx-1)) <= xmax(:,state_indx+12*(uav_indx-1))*ones(params.Nsteps, 1);
        constraints = [constraints; workspace_bds];
    end
end

%% Specification: reach (x,y) goal of both UAVs
x_lb1 = xr_uav1(4)-0.05;
x_ub1 = xr_uav1(4)+0.05;
y_lb1 = xr_uav1(5)-0.05;
y_ub1 = xr_uav1(5)+0.05;

x_lb2 = xr_uav2(4)-0.05;
x_ub2 = xr_uav2(4)+0.05;
y_lb2 = xr_uav2(5)-0.05;
y_ub2 = xr_uav2(5)+0.05;

x_pos1 = x(:,[4, 5]);
x_pos2 = x(:,[4+12, 5+12]);

[constr1, theta1] = theta_goal(x_pos1, x_lb1, x_ub1, y_lb1, y_ub1);
[constr2, theta2] = theta_goal(x_pos2, x_lb2, x_ub2, y_lb2, y_ub2);
constraints = [constraints; constr1; constr2];

% % Globally
a1 = params.N-10;
b1 = params.N;
[constr_g1, theta_g1] = quant_globally(theta1, 1, a1, b1);
constraints = [constraints; constr_g1];

[constr_g2, theta_g2] = quant_globally(theta2, 1, a1, b1);
constraints = [constraints; constr_g2];

%% Specification: be abs(z-z_goal)<=0.05 near the end for both UAVs
% Predicate Ax>=b;
z_lb1 = xr_uav1(6) - 0.05;
z_ub1 = xr_uav1(6) + 0.05;
z_lb2 = xr_uav2(6) - 0.05;
z_ub2 = xr_uav2(6) + 0.05;

z_pos1 = x(:,6);
z_pos2 = x(:,6+12);

[constr_z1, theta_z1] = theta_goal_single(z_pos1, z_lb1, z_ub1);
[constr_z2, theta_z2] = theta_goal_single(z_pos2, z_lb2, z_ub2);
constraints = [constraints; constr_z1; constr_z2];

% Finally Globally
a1 = params.N-10;
b1 = params.N;
[constr_gz1, theta_gz1] = quant_globally(theta_z1, 1, a1, b1);
constraints = [constraints; constr_gz1];

[constr_gz2, theta_gz2] = quant_globally(theta_z2, 1, a1, b1);
constraints = [constraints; constr_gz2];


%% Combine all thetas
[constr, theta_phi] = quant_and([theta_g2, theta_g1, theta_gz2, theta_gz1]);
% [constr, theta_phi] = quant_and([theta_gz2, theta_gz1]);
constraints = [constraints; constr];

%% Objective Function to maximise theta
objective = objective + theta_phi;

% options = sdpsettings('solver','mosek', 'verbose', 0);
options = sdpsettings('solver','gurobi', 'gurobi.TimeLimit', 520);
sol = optimize(constraints, -objective, options);
% sol = optimize(constraints, -objective);

%%
R = value(theta_phi);
fprintf('Right time rob: %i(time units)\n', R);
%
color_red = [200./255, 0, 0];
color_blue = [51./255, 102./255, 1];
color_green = [.0 .875 .0];
color_orange = [1 102./255 0];
%%%%%%

%% Simulate in closed loop
nsim = params.N;

% Preallocate state and input arrays
X = zeros(nx, params.Nsteps);
X(:, 1) = x0';
for i = 1 : nsim
    U = value(u(i, :))';
    X(:, i+1) = Ad*X(:, i) + Bd*U;
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
myVideo = VideoWriter('multi_3D_uav_time_rob_qp'); %open video file
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
    disp(['Time:', num2str(i), '   ', num2str(pos2(3,i))])
    plot3(pos1(1, 1:i), pos1(2, 1:i), pos1(3, 1:i), 'b', 'LineWidth', 1);
    plot3(pos1(1, i)+[cuav1_1(1), cuav1_2(1)], pos1(2, i)+[cuav1_1(2), cuav1_2(2)], pos1(3, i)+[cuav1_1(3), cuav1_2(3)], 'r', 'LineWidth', 2);
    plot3(pos1(1, i)+[cuav1_3(1), cuav1_4(1)], pos1(2, i)+[cuav1_3(2), cuav1_4(2)], pos1(3, i)+[cuav1_3(3), cuav1_4(3)], 'g', 'LineWidth', 2);
    plot3(pos1(1, i)+[cuav1_5(1), cuav1_6(1)], pos1(2, i)+[cuav1_5(2), cuav1_6(2)], pos1(3, i)+[cuav1_5(3), cuav1_6(3)], 'b', 'LineWidth', 2);

    plot3(pos2(1, 1:i), pos2(2, 1:i), pos2(3, 1:i), 'b', 'LineWidth', 1);
    plot3(pos2(1, i)+[cuav2_1(1), cuav2_2(1)], pos2(2, i)+[cuav2_1(2), cuav2_2(2)], pos2(3, i)+[cuav2_1(3), cuav2_2(3)], 'r', 'LineWidth', 2);
    plot3(pos2(1, i)+[cuav2_3(1), cuav2_4(1)], pos2(2, i)+[cuav2_3(2), cuav2_4(2)], pos2(3, i)+[cuav2_3(3), cuav2_4(3)], 'g', 'LineWidth', 2);
    plot3(pos2(1, i)+[cuav2_5(1), cuav2_6(1)], pos2(2, i)+[cuav2_5(2), cuav2_6(2)], pos2(3, i)+[cuav2_5(3), cuav2_6(3)], 'b', 'LineWidth', 2);

    xlim([-2.25, 2.25]);
    ylim([-2.25, 2.25]);
    zlim([-0.5, 2.5]);
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
