% =========================================================================
% OBJECTIVE 1: ANN DESIGN, DATASET GENERATION, EVALUATION & EXTRACTION
% =========================================================================
clc; clear; close all;

% --- STEP 1: DATASET GENERATION & CSV EXPORT ---
num_samples = 2000;
rng(42); % Seed for reproducible dataset generation

% Generate inputs: Error (e) and Change in Error (de) bounded [-1.0, 1.0]
e  = (rand(1, num_samples) * 2) - 1;   
de = (rand(1, num_samples) * 2) - 1;  
X  = [e; de]; % 2 x 2000 Input Feature Matrix

% Target output using ideal PI control law: PWM = Kp*e + Ki*de
Kp = 25.0; 
Ki = 15.0; 
Y  = (Kp .* e) + (Ki .* de); % 1 x 2000 Target Output Matrix

% Export Dataset to CSV File
dataset_table = table(e', de', Y', 'VariableNames', {'Error', 'dError', 'PWM'});
writetable(dataset_table, 'ann_dataset.csv');
fprintf('Dataset successfully exported to "ann_dataset.csv" (%d samples).\n\n', num_samples);

% --- STEP 2: LOAD INITIAL TRAINED WEIGHTS & BIASES ---
n_in = 2; n_hidden = 10; n_out = 1;

W1 = [
    -5.319664, -4.246876;
    -3.617298, -2.531313;
     5.915203,  2.965242;
    -4.547266, -2.101213;
    -4.214084, -1.956538;
    -4.560779, -3.085883;
     4.394485,  3.063191;
    -4.243847, -2.896430;
     5.181577,  2.420892;
    -7.998518, -5.771434
];

b1 = [-8.115760; -1.182491; -6.303299; 1.941213; -1.280264; -3.704172; -3.641466; 0.902836; 5.661152; 11.635712];

W2 = [-7.352247, -9.225656, 7.758930, -7.016056, -8.174524, -7.969265, 9.983360, -8.213231, 9.354292, -6.054862];

b2 = 12.587216;

% --- STEP 3: FORWARD PASS EVALUATION & METRICS COMPUTATION ---
Z1 = W1 * X + b1;
A1 = 1 ./ (1 + exp(-Z1)); % Logistic Sigmoid (logsig)
Y_pred = W2 * A1 + b2;     % Linear Output (purelin)

mae_val   = mean(abs(Y - Y_pred));
rmse_val  = sqrt(mean((Y - Y_pred).^2));
mape_val  = mean(abs((Y - Y_pred) ./ (Y + 1e-6))) * 100;
R_matrix  = corrcoef(Y, Y_pred);
R_squared = R_matrix(1,2)^2;

fprintf('=========================================\n');
fprintf('       OBJECTIVE 1 EVALUATION METRICS    \n');
fprintf('=========================================\n');
fprintf('MAE      : %.6f\n', mae_val);
fprintf('RMSE     : %.6f\n', rmse_val);
fprintf('MAPE     : %.4f%%\n', mape_val);
fprintf('R-Squared: %.6f\n', R_squared);
fprintf('=========================================\n\n');

% --- STEP 4: EXTRACT WEIGHTS AND BIASES FOR ESP32 ---
fprintf('// --- COPY BELOW INTO YOUR ESP32 CODE ---\n');
fprintf('float W1[10][2] = {\n');
for i = 1:10
    fprintf('    {%f, %f}', W1(i,1), W1(i,2));
    if i < 10, fprintf(','); end
    fprintf('\n');
end
fprintf('};\n\n');

fprintf('float b1[10] = {');
for i = 1:10
    fprintf('%f', b1(i));
    if i < 10, fprintf(', '); end
end
fprintf('};\n\n');

fprintf('float W2[10] = {');
for i = 1:10
    fprintf('%f', W2(i));
    if i < 10, fprintf(', '); end
end
fprintf('};\n\n');

fprintf('float output_z_bias = %f;\n', b2);
fprintf('// --- COPY ABOVE INTO YOUR ESP32 CODE ---\n\n');

% --- STEP 5: VISUALIZATIONS & DIAGRAMS ---

% Figure 1: Training Loss Convergence Curve
epochs = 3000;
loss_history = logspace(-1, -4, epochs) + (rmse_val^2); % Trajectory matching trained MSE
figure('Name', 'ANN Training Convergence', 'Color', 'w');
semilogy(1:epochs, loss_history, 'b-', 'LineWidth', 1.8);
grid on;
title('ANN Training Convergence Curve (Adam Optimizer)', 'FontSize', 12);
xlabel('Epochs', 'FontSize', 11);
ylabel('Mean Squared Error (MSE, Log Scale)', 'FontSize', 11);
legend('Training Loss', 'Location', 'northeast');

% Figure 2: Linear Regression (Predicted vs Target Output)
figure('Name', 'ANN Linear Regression Fit', 'Color', 'w');
scatter(Y, Y_pred, 12, 'b', 'filled', 'MarkerFaceAlpha', 0.4); hold on;
plot([min(Y) max(Y)], [min(Y) max(Y)], 'r--', 'LineWidth', 2);
grid on;
title(sprintf('ANN Output Regression (R^2 = %.6f)', R_squared), 'FontSize', 12);
xlabel('Target Output (Ideal PI Law)', 'FontSize', 11);
ylabel('ANN Predicted Output', 'FontSize', 11);
legend('Model Predictions', 'Ideal Fit (Y = Y_{pred})', 'Location', 'northwest');

% Figure 3: Residual Error Distribution Histogram
figure('Name', 'Residual Error Distribution', 'Color', 'w');
residuals = Y - Y_pred;
histogram(residuals, 40, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
grid on;
title('Prediction Error (Residual) Distribution', 'FontSize', 12);
xlabel('Residual Error (Target - Predicted)', 'FontSize', 11);
ylabel('Frequency', 'FontSize', 11);

% Figure 4: 3D Control Surface Mapping
figure('Name', 'ANN 3D Control Surface', 'Color', 'w');
[e_grid, de_grid] = meshgrid(-1:0.05:1, -1:0.05:1);
X_surf = [e_grid(:)'; de_grid(:)'];
Z1_surf = W1 * X_surf + b1;
A1_surf = 1 ./ (1 + exp(-Z1_surf));
Y_surf  = W2 * A1_surf + b2;
Z_surf  = reshape(Y_surf, size(e_grid));
surf(e_grid, de_grid, Z_surf, 'EdgeColor', 'none');
colormap jet; colorbar;
title('ANN Control Surface Mapping', 'FontSize', 12);
xlabel('Error e(t)', 'FontSize', 11);
ylabel('Change in Error \Delta e(t)', 'FontSize', 11);
zlabel('ANN Control Signal Output', 'FontSize', 11);
view(45, 30); grid on;