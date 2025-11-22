%% Plot Experimental Data from Serial Monitor
% This script reads and plots CSV data from Arduino Serial Monitor
%
% Authors: BUET EEE Group 03
% Usage: Save serial data as CSV file and update filename below

clear all;
close all;
clc;

%% Configuration

% CSV file name (change this to your file)
filename = 'serial_data.csv';

% Check if file exists
if ~isfile(filename)
    error('File not found: %s\nPlease save Serial Monitor data as CSV first.', filename);
end

%% Load Data

fprintf('Loading data from: %s\n', filename);

% Read CSV data
% Format: Timestamp(ms), Setpoint(cm), Position(cm)
data = readmatrix(filename);

% Extract columns
timestamp_ms = data(:, 1);
setpoint_cm = data(:, 2);
position_cm = data(:, 3);

% Convert timestamp to seconds
time_s = timestamp_ms / 1000;

% Calculate error
error_cm = position_cm - setpoint_cm;

fprintf('Data loaded successfully!\n');
fprintf('Total samples: %d\n', length(time_s));
fprintf('Duration: %.2f seconds\n', max(time_s));

%% Plot Response

figure('Position', [100, 100, 1400, 900]);

% Main Response Plot
subplot(3, 1, 1);
plot(time_s, setpoint_cm, 'r--', 'LineWidth', 2);
hold on;
plot(time_s, position_cm, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (cm)');
title('Ball and Beam System Response');
legend('Setpoint', 'Car Position', 'Location', 'best');
grid on;
xlim([0 max(time_s)]);

% Error Plot
subplot(3, 1, 2);
plot(time_s, error_cm, 'g-', 'LineWidth', 1.5);
hold on;
yline(0, 'r--', 'Zero Error');
xlabel('Time (s)');
ylabel('Error (cm)');
title('Position Error vs. Time');
grid on;
xlim([0 max(time_s)]);

% Error Histogram
subplot(3, 1, 3);
histogram(error_cm, 30, 'FaceColor', [0.3 0.7 0.9]);
xlabel('Error (cm)');
ylabel('Frequency');
title('Error Distribution');
grid on;

%% Performance Metrics

% Final 20% of data (steady-state)
steady_start = round(0.8 * length(time_s));
steady_error = error_cm(steady_start:end);
steady_position = position_cm(steady_start:end);

% Calculate metrics
fprintf('\n=== Performance Metrics ===\n');
fprintf('Steady-State Error (Mean): %.4f cm\n', mean(steady_error));
fprintf('Steady-State Error (Std): %.4f cm\n', std(steady_error));
fprintf('Final Position: %.3f cm\n', position_cm(end));
fprintf('Target Setpoint: %.3f cm\n', setpoint_cm(end));
fprintf('Absolute Error: %.3f cm\n', abs(position_cm(end) - setpoint_cm(end)));
fprintf('Percentage Error: %.2f%%\n', ...
        100 * abs(position_cm(end) - setpoint_cm(end)) / setpoint_cm(end));

% Find peaks
[pks, locs] = findpeaks(position_cm);
if ~isempty(pks)
    [max_peak, max_idx] = max(pks);
    peak_time = time_s(locs(max_idx));
    overshoot = 100 * (max_peak - setpoint_cm(end)) / setpoint_cm(end);
    
    fprintf('\nPeak Value: %.3f cm\n', max_peak);
    fprintf('Peak Time: %.3f s\n', peak_time);
    fprintf('Overshoot: %.2f%%\n', overshoot);
end

% Settling time (within 2% of final value)
final_value = position_cm(end);
tolerance = 0.02 * final_value;
settling_idx = find(abs(position_cm - final_value) > tolerance, 1, 'last');
if ~isempty(settling_idx)
    settling_time = time_s(settling_idx);
    fprintf('Settling Time (2%% criterion): %.3f s\n', settling_time);
end

%% Save Figure

saveas(gcf, 'experimental_response.png');
fprintf('\nFigure saved as: experimental_response.png\n');
