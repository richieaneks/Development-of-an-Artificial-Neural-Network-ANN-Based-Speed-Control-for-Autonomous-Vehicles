% =========================================================================
% UNIFIED EVALUATION & METRIC DIAGRAM GENERATOR FOR ANN SPEED CONTROLLER
% =========================================================================
clc; clear; close all;

% --- 1. LOAD TELEMETRY DATA ---
data = readtable('telemetry.csv'); % Expects columns: Error, dError, PWM
error_val = data.Error;
pwm_signal = data.PWM;

target = 0.50;               % Reference speed in m/s
actual = target - error_val; % Actual Speed = Target - Error
dt = 0.05;                   % 50 ms sampling interval
t = (0:length(actual)-1)' * dt;

% --- 2. CALCULATE TRANSIENT RESPONSE METRICS ---
[max_speed, max_idx] = max(actual);
Peak_Time = t(max_idx);
Overshoot_percent = ((max_speed - target) / target) * 100;

% Rise Time (Time to go from 10% to 90% of target)
idx_10 = find(actual >= 0.1 * target, 1);
idx_90 = find(actual >= 0.9 * target, 1);
Rise_Time = t(idx_90) - t(idx_10);

% Settling Time (Time to remain within +-5% of target)
settling_band = 0.05 * target;
upper_limit = target + settling_band;
lower_limit = target - settling_band;

is_outside = abs(actual - target) > settling_band;
last_outside_idx = find(is_outside, 1, 'last');

if isempty(last_outside_idx)
    Settling_Time = 0; % Always stayed inside the band
elseif last_outside_idx == length(t)
    Settling_Time = NaN; % Never settled within the time limit
    fprintf('Warning: System did not settle within the run duration.\n');
else
    Settling_Time = t(last_outside_idx + 1);
end

% --- 3. CALCULATE INTEGRAL ERROR METRICS OVER TIME ---
iae_cum  = cumsum(abs(error_val)) * dt;       % Cumulative IAE
ise_cum  = cumsum(error_val.^2) * dt;         % Cumulative ISE
itae_cum = cumsum(t .* abs(error_val)) * dt; % Cumulative ITAE

IAE  = iae_cum(end);
ISE  = ise_cum(end);
ITAE = itae_cum(end);

% --- 4. DISPLAY RESULTS IN COMMAND WINDOW ---
fprintf('\n--- Control System Performance Metrics ---\n');
fprintf('Rise Time:      %.3f seconds\n', Rise_Time);
fprintf('Peak Time:      %.3f seconds\n', Peak_Time);
fprintf('Overshoot:      %.2f %%\n', Overshoot_percent);
fprintf('Settling Time:  %.3f seconds\n', Settling_Time);
fprintf('IAE:            %.4f\n', IAE);
fprintf('ISE:            %.4f\n', ISE);
fprintf('ITAE:           %.4f\n', ITAE);
fprintf('------------------------------------------\n\n');

% =========================================================================
% DIAGRAM 1: ANNOTATED TRANSIENT STEP RESPONSE GRAPH
% =========================================================================
figure('Name', 'Annotated Step Response', 'Color', 'w');
plot(t, actual, 'b-', 'LineWidth', 2); hold on;

% Target & Settling Band Lines
yline(target, 'r--', 'Target (0.50 m/s)', 'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left');
yline(upper_limit, 'k:', '+5% Band', 'LineWidth', 1.2);
yline(lower_limit, 'k:', '-5% Band', 'LineWidth', 1.2);

% Annotate Rise Time
plot([t(idx_10) t(idx_90)], [actual(idx_10) actual(idx_90)], 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
line([t(idx_10) t(idx_90)], [target*0.1 target*0.9], 'Color', 'm', 'LineStyle', '--', 'LineWidth', 1.5);
text(t(idx_90)+0.1, actual(idx_90), sprintf(' Rise Time: %.3fs', Rise_Time), 'FontSize', 10, 'FontWeight', 'bold');

% Annotate Peak Time and Overshoot
plot(Peak_Time, max_speed, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
text(Peak_Time + 0.15, max_speed, sprintf(' Peak: %.2fm/s (%.1f%% Overshoot)', max_speed, Overshoot_percent), ...
    'FontSize', 10, 'FontWeight', 'bold');

% Annotate Settling Time
if ~isnan(Settling_Time) && Settling_Time > 0
    xline(Settling_Time, 'g--', sprintf(' Settling Time: %.3fs', Settling_Time), ...
        'LineWidth', 1.8, 'LabelVerticalAlignment', 'bottom');
end

xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Speed (m/s)', 'FontSize', 11);
title('Annotated Transient Step Response Dynamics', 'FontSize', 12);
legend('Actual Speed', 'Target Speed', '\pm5% Settling Band', 'Location', 'southeast');
grid on; hold off;

% =========================================================================
% DIAGRAM 2: CUMULATIVE INTEGRAL ERROR METRICS (IAE, ISE, ITAE)
% =========================================================================
figure('Name', 'Integral Error Accumulation', 'Color', 'w');

subplot(3, 1, 1);
plot(t, iae_cum, 'm-', 'LineWidth', 1.8); grid on;
ylabel('IAE');
title(sprintf('Integral Absolute Error (IAE Total = %.4f)', IAE), 'FontSize', 11);

subplot(3, 1, 2);
plot(t, ise_cum, 'c-', 'LineWidth', 1.8); grid on;
ylabel('ISE');
title(sprintf('Integral Squared Error (ISE Total = %.4f)', ISE), 'FontSize', 11);

subplot(3, 1, 3);
plot(t, itae_cum, 'r-', 'LineWidth', 1.8); grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('ITAE');
title(sprintf('Integral Time-Weighted Absolute Error (ITAE Total = %.4f)', ITAE), 'FontSize', 11);

% =========================================================================
% DIAGRAM 3: CONTROLLER ACTION & SYSTEM RESPONSE
% =========================================================================
figure('Name', 'Controller PWM vs Speed', 'Color', 'w');

% Left Y-Axis: Speed
yyaxis left
h1 = plot(t, actual, 'b-', 'LineWidth', 2); hold on;
h2 = yline(target, 'r--', 'LineWidth', 1.8);
ylabel('Speed (m/s)', 'FontSize', 11);

% Right Y-Axis: PWM
yyaxis right
h3 = plot(t, pwm_signal, 'm-', 'LineWidth', 1.5);
ylabel('ANN Output PWM (0 - 255)', 'FontSize', 11);

xlabel('Time (seconds)', 'FontSize', 11);
title('ANN Controller Effort (PWM) vs. Actual Vehicle Speed', 'FontSize', 12);
grid on;

% Dual-Axis Consolidated Legend
legend([h1, h2, h3], ...
    {'Actual Speed (m/s)', 'Target Reference Speed (0.50 m/s)', 'ANN PWM Control Output'}, ...
    'Location', 'southeast', 'FontSize', 10);

% =========================================================================
% DIAGRAM 4: PERFORMANCE METRICS DASHBOARD BAR CHART
% =========================================================================
figure('Name', 'Control Performance Dashboard', 'Color', 'w');

subplot(1, 2, 1);
b1 = bar([Rise_Time, Peak_Time, Settling_Time], 'FaceColor', 'flat');
b1.CData(1,:) = [0.2 0.6 0.8];
b1.CData(2,:) = [0.9 0.4 0.2];
b1.CData(3,:) = [0.3 0.7 0.3];
set(gca, 'XTickLabel', {'Rise Time (s)', 'Peak Time (s)', 'Settling Time (s)'});
ylabel('Time (seconds)', 'FontSize', 11);
title('Transient Time Metrics', 'FontSize', 11);
grid on;

subplot(1, 2, 2);
b2 = bar([IAE, ISE, ITAE], 'FaceColor', 'flat');
b2.CData(1,:) = [0.7 0.3 0.7];
b2.CData(2,:) = [0.2 0.7 0.7];
b2.CData(3,:) = [0.8 0.2 0.2];
set(gca, 'XTickLabel', {'IAE', 'ISE', 'ITAE'});
ylabel('Error Index Value', 'FontSize', 11);
title('Integral Error Criteria', 'FontSize', 11);
grid on;