clear; clc; close all;
%% 1) Define System Parameters
m1 = 2500;   % Sprung mass (vehicle body)
m2 = 320;    % Unsprung mass (wheel/tire)
k1 = 80000;  % Spring constant of the suspension
k2 = 500000; % Spring constant of the tire
b1 = 350;    % Damping coefficient of the suspension
b2 = 15020;  % Damping coefficient of the tire
% Set D = 0 for all outputs in this example
D = 0;
%% 2) Define State-Space Matrices (Two-Input Version)
% States: x = [ x1, x1_dot, x2, x2_dot ]'
% x1 = displacement of the body, x2 = displacement of the wheel
%
% B has two columns:
%   - Column 1 = Force from the actuator (u1)
%   - Column 2 = Force from road disturbances (u2)
A = [  0,      1,        0,       0;
     -32, -0.14,      32,    0.14;
      0,      0,        0,       1;
    250,      0,  -1812.5, -48.03125 ];
B = [ 0,          0;
     1/2500,     0;
     0,          0;
    -1/320, 1609.4375 ];
C = [1, -1, 0, 0];  % Output = difference in displacement (x1 - x2)
%% 3) Design LQR Controller
% Weighting matrices for state and control
Q = diag([5000, 500, 500, 100]);  % State weights
R = 10;                           % Control effort weight
% Calculate LQR gain for the actuator force channel = B(:,1)
K = lqr(A, B(:,1), Q, R);
%% 4) Simulation Parameters
T = 0:0.01:50;  % Time vector for simulation (50 seconds)
W = 0.5;        % Amplitude of step disturbance
% Initial state with a slight displacement
x0 = [0.1; 0; 0; 0];
%% 5) Open-loop Simulation
% Open-loop to observe the effect of road disturbance alone
Ac_open_loop = A;
Bc_open_loop = B(:,2);  % Input from road disturbance
Cc_open_loop = C;
Dc_open_loop = D;
% Input for step disturbance matches Bc_open_loop
U_open_loop = W * ones(length(T), 1);
% Define the open-loop state-space system
sys_open_loop = ss(Ac_open_loop, Bc_open_loop, Cc_open_loop, Dc_open_loop);
% Simulate the open-loop response
[y_open_loop, t_open_loop, x_open_loop] = lsim(sys_open_loop, U_open_loop, T, x0);
%% 6) Closed-loop Simulation
% Apply internal actuator force as a control input
Ac_closed_loop = (A - B(:,1)*K); % Modify A matrix by subtracting the control contribution
Bc_closed_loop = B(:,2);         % Keep the road disturbance input
Cc_closed_loop = C;
Dc_closed_loop = D;
% Input for step disturbance remains the same
U_closed_loop = W * ones(length(T), 1);
% Define the closed-loop state-space system
sys_cl = ss(Ac_closed_loop, Bc_closed_loop, Cc_closed_loop, Dc_closed_loop);
% Simulate the closed-loop response
[y_closed_loop, t_closed_loop, x_closed_loop] = lsim(sys_cl, U_closed_loop, T, x0);
%% 7) Calculate Control Effort
% Compute the control force at each time step
u_closed_loop = -K * x_closed_loop';
%% 8) Plotting Results
% Open-loop Response
figure;
subplot(2,1,1);
plot(t_open_loop, y_open_loop, 'b', 'LineWidth', 1.5); % Changed color to blue
xlabel('Time (s)');
ylabel('y_1 = x_1 - x_2 (m)');
title('Open-loop: Suspension Displacement Difference');
grid on;
% Closed-loop Response
subplot(2,1,2);
plot(t_closed_loop, y_closed_loop, 'r', 'LineWidth', 1.5); % Changed color to red
xlabel('Time (s)');
ylabel('y_1 = x_1 - x_2 (m)');
title('Closed-loop: Suspension Displacement Difference');
grid on;
% Control Effort
figure;
plot(t_closed_loop, u_closed_loop, 'g', 'LineWidth', 1.5); % Changed color to green
xlabel('Time (s)');
ylabel('Control Force (N)');
title('Control Effort (u) with LQR Control');
grid on;