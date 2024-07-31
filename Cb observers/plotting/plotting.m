clear all;
% Define the filenames of the CSV files
file1 = 'data_filtered/owen_calculated.csv'; % Calculated
file2 = 'data_filtered/owen_EKF_filtered.csv'; % EKF Observerd
file3 = 'data_filtered/owen_UKF_filtered.csv'; % UKF Observed

% Read the data from the CSV files
data1 = readmatrix(file1)';
data2 = readmatrix(file2)';
data3 = readmatrix(file3)';

% Extract time and estimated data
times1 = data1(:, 1);
x_est1 = data1(:, 2);
times2 = data2(:, 1);
x_est2 = data2(:, 2);
times3 = data3(:, 1);
x_est3 = data3(:, 2);

% Define the time range of interest for cropping
time_min = 9;
time_max = 17;

% Crop the data based on the defined time range
index1 = times1 >= time_min & times1 <= time_max;
times1 = times1(index1);
x_est1 = x_est1(index1);

index2 = times2 >= time_min & times2 <= time_max;
times2 = times2(index2);
x_est2 = x_est2(index2);

index3 = times3 >= time_min & times3 <= time_max;
times3 = times3(index3);
x_est3 = x_est3(index3);

% Downsample data to reduce the number of points
factor = 40; % Adjust factor to downsample more or less
times1 = times1(1:factor:end);
x_est1 = x_est1(1:factor:end);
times2 = times2(1:factor:end);
x_est2 = x_est2(1:factor:end);
times3 = times3(1:factor:end);
x_est3 = x_est3(1:factor:end);

% Apply smoothing to the data
windowSize = 1; % Adjust window size for more or less smoothing
x_est1_smooth = movmean(x_est1, windowSize);
x_est2_smooth = movmean(x_est2, windowSize);
x_est3_smooth = movmean(x_est3, windowSize);

% Determine dynamic limits for x and y axes
x_min = min([times1; times2; times3]);
x_max = max([times1; times2; times3]);
y_min = min([x_est1; x_est2; x_est3]);
y_max = max([x_est1; x_est2; x_est3]);

% Create the plot
figure('Position', [100, 100, 800, 600]); % Set figure size

% Plot each dataset with different styles
plot(times1, x_est1_smooth, 'o-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Calculated'); hold on;
plot(times2, x_est2_smooth, 's--', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'EKF Observed');
plot(times3, x_est3_smooth, 'd-.', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'UKF Observed');

% Add labels and title
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Ballistic Coefficient (kg/m^2)', 'FontSize', 12, 'FontWeight', 'bold');

% Add a legend
legend('show', 'Location', 'northeast');

% Chart limits
xlim([x_min, x_max]);
ylim([y_min, y_max]);

% Format the axes with explicit tick values and readable format
set(gca, 'FontSize', 12, 'LineWidth', 1.5);
xticks(linspace(x_min, x_max, 10)); % Set x-ticks at intervals based on the range
yticks(linspace(y_min, y_max, 10)); % Set y-ticks at intervals based on the range

% Set tick label format to remove excessive decimal places
xtickformat('%.0f');
ytickformat('%.0f');

% Add grid lines
grid on;

% Enhance the appearance
set(gca, 'Box', 'on');
set(gcf, 'Color', 'w'); % Set background color to white

% Save the plot as a high-resolution image
saveas(gcf, 'IAC_Cb_Plot.png');
