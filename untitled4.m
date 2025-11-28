% Parameters
m1 = 2500; 
m2 = 320;
k1 = 80000; 
k2 = 500000;
b1 = 350; 
b2 = 15020;

% State-space matrices
A = [0, 1, 0, 0;
    -k1/m1, -b1/m1, k1/m1, b1/m1;
     0, 0, 0, 1;
     k1/m2, b1/m2, -(k1+k2)/m2, -(b1+b2)/m2];

B = [0, 0;
     1/m1, 0;
     0, 0;
     -1/m2, k2/m2 + b2/m2];

C = [1, 0, -1, 0];
D = [0, 0];

% Define the transfer functions
sys = ss(A, B, C, D); % State-space model
G1 = tf(sys(:, 1)); % Transfer function for U (control force)
G2 = tf(sys(:, 2)); % Transfer function for W (road disturbance)

% Step response for G1
figure;
step(G1);
title('Step Response of G1 (x1 - x2 / U)');
grid on;
info1 = stepinfo(G1); % Step response characteristics
fprintf('G1: Settling Time = %.2f s, Max Overshoot = %.2f%%\n', ...
        info1.SettlingTime, info1.Overshoot);

% Step response for G2
figure;
step(0.1 * G2); % W = 0.1 meters
title('Step Response of G2 (x1 - x2 / W)');
grid on;
info2 = stepinfo(0.1 * G2); % Step response characteristics
fprintf('G2: Settling Time = %.2f s, Max Overshoot = %.2f%%\n', ...
        info2.SettlingTime, info2.Overshoot);

% Damped Frequency Calculation
[wn1, zeta1] = damp(G1); % Natural frequency and damping ratio for G1
[wn2, zeta2] = damp(G2); % Natural frequency and damping ratio for G2
fprintf('G1: Damped Frequency = %.2f Hz\n', wn1(1)/2/pi);
fprintf('G2: Damped Frequency = %.2f Hz\n', wn2(1)/2/pi);