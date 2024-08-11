% Set LaTeX as the default interpreter for text, axes, and legends
set(groot,'defaulttextinterpreter','latex');  
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaultLegendInterpreter','latex');

% Define the filter types and other settings
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", "EKF_constant_acceleration", "EKF_constant_Cb"];
time_min = 9;
time_max = 17;
factor = 40; % Downsampling factor
windowSize = 1; % Smoothing window size

% Initialize cell arrays for storing the processed data for each filter
all_times = cell(1, length(filters));
all_x_est_smooth = cell(1, length(filters));
all_apogee_est_smooth = cell(1, length(filters));

% Loop through each filter to process and store the corresponding data
for i = 1:length(filters)
    filter_name = filters(i);
    data_filename = strcat('data_filtered/', filter_name, '_filtered_data.csv');
    
    % Import the data
    data = csvread(data_filename, 1, 0); % Skips the header row
    times = data(:, 1); % First column: time
    
    % Ballistic Coefficient (5th Column)
    x_est = data(:, 5); % 5th column: estimated ballistic coefficient
    index = times >= time_min & times <= time_max; % Time filter
    times_filtered = times(index);
    x_est_filtered = x_est(index);
    times_sampled = times_filtered(1:factor:end); % Downsampled time data
    x_est_sampled = x_est_filtered(1:factor:end); % Downsampled ballistic coefficient
    x_est_smooth = movmean(x_est_sampled, windowSize); % Smoothed ballistic coefficient
    all_times{i} = times_sampled; % Store the sampled times
    all_x_est_smooth{i} = x_est_smooth; % Store the smoothed ballistic coefficient
    
    % Estimated Apogee (6th Column)
    apogee_est = data(:, 6); % 6th column: estimated apogee
    apogee_est_filtered = apogee_est(index);
    apogee_est_sampled = apogee_est_filtered(1:factor:end); % Downsampled apogee estimate
    apogee_est_smooth = movmean(apogee_est_sampled, windowSize); % Smoothed apogee estimate
    all_apogee_est_smooth{i} = apogee_est_smooth; % Store the smoothed apogee estimate
end

% Determine plot limits
x_min = min(cellfun(@(x) min(x), all_times));
x_max = max(cellfun(@(x) max(x), all_times));
y_min_bc = min(cellfun(@(y) min(y), all_x_est_smooth));
y_max_bc = max(cellfun(@(y) max(y), all_x_est_smooth));
y_min_apogee = min(cellfun(@(y) min(y), all_apogee_est_smooth));
y_max_apogee = max(cellfun(@(y) max(y), all_apogee_est_smooth));

% Define markers and colors
% Markers: Circles for UKF, Squares for EKF
% Colors: Differentiated by constant acceleration vs constant Cb
markers = {'o', 'o', 's', 's'};
colors = {
    [0 0.4470 0.7410], % UKF Constant Acceleration
    [0.8500 0.3250 0.0980], % UKF Constant Cb
    [0 0.4470 0.7410], % EKF Constant Acceleration
    [0.8500 0.3250 0.0980]  % EKF Constant Cb
};

% Plot Ballistic Coefficient
figure('Position', [100, 100, 700, 500], "Name", "Ballistic Coefficient Comparison Plot");

% Initialize an array to store plot handles for the ballistic coefficient
plot_handles_bc = gobjects(1, length(filters));

for i = 1:length(filters)
    % Plot and store the handle for the ballistic coefficient
    plot_handles_bc(i) = plot(all_times{i}, all_x_est_smooth{i}, [markers{i}, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', colors{i}, ...
        'DisplayName', strrep(filters(i), '_', '\_')); 
    hold on;
end

% Add labels, legend, and customize the ballistic coefficient plot
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Ballistic Coefficient $(kg/m^2)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(plot_handles_bc, strrep(filters, '_', '\_'), 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Arial');

xlim([x_min, x_max]);
ylim([y_min_bc, y_max_bc]);

set(gca, 'FontSize', 10, 'LineWidth', 1.5, 'FontName', 'Arial');
xticks(linspace(x_min, x_max, 10)); 
yticks(linspace(y_min_bc, y_max_bc, 10)); 

xtickformat('%.0f');
ytickformat('%.0f');

grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

set(gca, 'Box', 'on');
set(gcf, 'Color', 'w');

saveas(gcf, 'plots/IAC_Cb_Plot_IAC.png');

%% Plot Estimated Apogee
% Plot Estimated Apogee
figure('Position', [100, 100, 700, 500], "Name", "Predicted Apogee Comparison Plot");

% Initialize an array to store plot handles for the apogee estimate
plot_handles_apogee = gobjects(1, length(filters));

for i = 1:length(filters)
    % Plot and store the handle for the apogee estimate
    plot_handles_apogee(i) = plot(all_times{i}, all_apogee_est_smooth{i}, [markers{i}, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', colors{i}, ...
        'DisplayName', strrep(filters(i), '_', '\_')); 
    hold on;
end

% Add the vertical line for true apogee
true_apogee_x = 791.5; % Replace with the x-value where you want the line
hline = yline(true_apogee_x, '--k', 'LineWidth', 1.5);

% Add labels, legend, and customize the apogee estimate plot
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Estimated Apogee Error $(m)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');

% Create legend including the vertical line
legend([plot_handles_apogee, hline], [strrep(filters, '_', '\_'), 'True Apogee'], ...
    'Location', 'best', 'FontSize', 10, 'FontName', 'Arial');

% Ensure the legend has a box
set(lgd, 'Box', 'on', 'EdgeColor', 'k'); % 'EdgeColor', 'k' ensures the box is black

xlim([x_min, x_max]);
ylim([y_min_apogee, y_max_apogee]);

set(gca, 'FontSize', 10, 'LineWidth', 1.5, 'FontName', 'Arial');
xticks(linspace(x_min, x_max, 10)); 
yticks(linspace(y_min_apogee, y_max_apogee, 10)); 

xtickformat('%.0f');
ytickformat('%.0f');

grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

set(gca, 'Box', 'on');
set(gcf, 'Color', 'w');

saveas(gcf, 'plots/IAC_Apogee_Plot_IAC.png');
