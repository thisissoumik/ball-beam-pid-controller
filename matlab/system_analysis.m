%% Ball and Beam PID Controller - System Analysis
% This script analyzes the ball and beam control system performance
%
% Authors: BUET EEE Group 03
% Course: EEE 318 - Control Systems Laboratory
% Date: December 2024

clear all;
close all;
clc;

%% System Parameters

% PID Controller Parameters (Ziegler-Nichols Tuned)
Kp = 8.4;               % Proportional gain
Ki = 3.73;              % Integral gain
Kd = 8.0;               % Derivative gain

% Servo Motor Parameters (MG996R)
K_servo = 181.82;       % Servo gain
tau_servo = 0.04255;    % Servo time constant (s)

% Ball and Beam Parameters
m = 0.02;               % Ball mass (kg)
L = 0.3;                % Beam length (m)
delta = 0.08;           % Lever arm offset (m)
fv = 0.02;              % Friction coefficient
g = 9.81;               % Gravity (m/s^2)

%% Transfer Functions

s = tf('s');

% PID Controller
G_PID = Kp + Ki/s + Kd*s;
fprintf('PID Controller Transfer Function:\n');
disp(G_PID);

% Servo Motor (First-order system)
G_servo = K_servo / (tau_servo*s + 1);
fprintf('\nServo Motor Transfer Function:\n');
disp(G_servo);

% Ball and Beam System
% G(s) = (2*m*g*delta/L) / (m*s^2 + fv*s)
G_beam = (2*m*g*delta/L) / (m*s^2 + fv*s);
fprintf('\nBall and Beam Transfer Function:\n');
disp(G_beam);

% Open-Loop Transfer Function
G_open = G_PID * G_servo * G_beam;
fprintf('\nOpen-Loop Transfer Function:\n');
disp(G_open);

% Closed-Loop Transfer Function
G_closed = feedback(G_open, 1);
fprintf('\nClosed-Loop Transfer Function:\n');
disp(G_closed);

%% Step Response Analysis

figure('Position', [100, 100, 1200, 800]);

% Open-Loop Step Response
subplot(2, 2, 1);
step(G_open);
title('Open-Loop Step Response');
grid on;

% Closed-Loop Step Response
subplot(2, 2, 2);
[y, t] = step(G_closed);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;
plot([0 max(t)], [1 1], 'r--', 'LineWidth', 1.5);
title('Closed-Loop Step Response');
xlabel('Time (s)');
ylabel('Position (normalized)');
legend('Response', 'Setpoint');
grid on;

% Step Response Info
info = stepinfo(G_closed);
fprintf('\n=== Step Response Characteristics ===\n');
fprintf('Rise Time: %.3f s\n', info.RiseTime);
fprintf('Settling Time: %.3f s\n', info.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);
fprintf('Peak: %.3f\n', info.Peak);
fprintf('Peak Time: %.3f s\n', info.PeakTime);

%% Root Locus

subplot(2, 2, 3);
rlocus(G_open);
title('Root Locus');
grid on;

%% Bode Plot

subplot(2, 2, 4);
bode(G_open);
title('Bode Plot (Open-Loop)');
grid on;

% Calculate Margins
[Gm, Pm, Wcg, Wcp] = margin(G_open);
fprintf('\n=== Stability Margins ===\n');
fprintf('Gain Margin: %.2f dB (at %.2f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('Phase Margin: %.2f deg (at %.2f rad/s)\n', Pm, Wcp);

%% Pole-Zero Map

figure('Position', [100, 100, 800, 600]);
pzmap(G_closed);
title('Pole-Zero Map (Closed-Loop)');
grid on;

% Check Stability
poles_closed = pole(G_closed);
fprintf('\n=== Closed-Loop Poles ===\n');
disp(poles_closed);

if all(real(poles_closed) < 0)
    fprintf('System is STABLE (all poles in LHP)\n');
else
    fprintf('System is UNSTABLE (poles in RHP)\n');
end

%% Frequency Response

figure('Position', [100, 100, 1200, 400]);

% Nyquist Plot
subplot(1, 2, 1);
nyquist(G_open);
title('Nyquist Plot');
grid on;

% Nichols Chart
subplot(1, 2, 2);
nichols(G_open);
title('Nichols Chart');
grid on;

%% Save Results

fprintf('\n=== Analysis Complete ===\n');
fprintf('Results saved to workspace\n');

% Save important variables
save('system_analysis_results.mat', 'G_PID', 'G_servo', 'G_beam', ...
     'G_open', 'G_closed', 'info', 'Gm', 'Pm', 'Wcg', 'Wcp');
