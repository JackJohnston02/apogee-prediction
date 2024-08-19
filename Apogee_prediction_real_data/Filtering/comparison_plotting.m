% Set default text interpreter to LaTeX for better formatting in plots
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

% List of filters, including CKF, for comparison
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", ...
           "EKF_constant_acceleration", "EKF_constant_Cb", ...
           "CKF_constant_acceleration", "CKF_constant_Cb"];

% Define symbols and colors for the different model types
constant_accel_markers = 'o'; % Symbol for constant acceleration models
constant_cb_markers = 's';    % Symbol for constant ballistic coefficient models

% Define colors corresponding to each filter
colors = {
    [0 0.4470 0.7410],  % Color for UKF_constant_acceleration
    [0 0.4470 0.7410],  % Color for UKF_constant_Cb
    [0.8500 0.3250 0.0980],  % Color for EKF_constant_acceleration
    [0.8500 0.3250 0.0980],  % EKF_constant_Cb
    [0.9290 0.6940 0.1250],  % CKF_constant_acceleration
    [0.9290 0.6940 0.1250]   % CKF_constant_Cb
    };

% Define time range and other parameters
time_min = 9;
time_max = 23;
factor = 40;
windowSize = 10;

% Initialize cells to store time and smoothed estimates for each filter
all_times = cell(1, length(filters));
all_x_est_smooth = cell(1, length(filters));
all_apogee_est_smooth = cell(1, length(filters));

% Loop through each filter to process and store data
for i = 1:length(filters)
    filter_name = filters(i);
    data_filename = strcat('data_filtered/', filter_name, '_filtered_data.csv');

    % Read the data from the corresponding CSV file
    data = csvread(data_filename, 1, 0);
    times = data(:, 1);
    x_est = data(:, 5);

    % Filter data based on the specified time range
    index = times >= time_min & times <= time_max;
    times_filtered = times(index);
    x_est_filtered = x_est(index);

    % Downsample and smooth the data
    times_sampled = times_filtered(1:factor:end);
    x_est_sampled = x_est_filtered(1:factor:end);
    x_est_smooth = movmean(x_est_sampled, windowSize);
    all_times{i} = times_sampled;
    all_x_est_smooth{i} = x_est_smooth;

    % Process the apogee estimate
    apogee_est = data(:, 6);
    apogee_est_filtered = apogee_est(index);
    apogee_est_sampled = apogee_est_filtered(1:factor:end);
    apogee_est_smooth = movmean(apogee_est_sampled, windowSize);
    all_apogee_est_smooth{i} = apogee_est_smooth;

    % Store the true apogee from the UKF_constant_acceleration filter
    if filter_name == "UKF_constant_acceleration"
        true_apogee = apogee_est_smooth(end);
    end
end

% Determine plot limits based on the processed data
x_min = min(cellfun(@(x) min(x), all_times));
x_max = max(cellfun(@(x) max(x), all_times));
y_min_bc = min(cellfun(@(y) min(y), all_x_est_smooth));
y_max_bc = max(cellfun(@(y) max(y), all_x_est_smooth));
y_min_apogee = round(true_apogee, -1) - 25; % Min y-limit for apogee plot
y_max_apogee = round(true_apogee, -1) + 25; % Max y-limit for apogee plot

%% Plot for Ballistic Coefficient Comparison
figure('Position', [100, 100, 700, 500], "Name", "Ballistic Coefficient Comparison Plot");

% Initialize plot handles for each filter
plot_handles_bc = gobjects(1, length(filters));

% Plot ballistic coefficient estimates for each filter
for i = 1:length(filters)
    if contains(filters(i), 'constant_acceleration')
        marker = constant_accel_markers; % Use circle for constant acceleration models
    else
        marker = constant_cb_markers;    % Use square for constant ballistic coefficient models
    end
    plot_handles_bc(i) = plot(all_times{i}, all_x_est_smooth{i}, [marker, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', colors{i}, ...
        'DisplayName', strrep(filters(i), '_', '\_')); % Display name with LaTeX formatting
    hold on;
end

% Add labels, legend, and axis formatting
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Ballistic Coefficient $(kg/m^2)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(plot_handles_bc, strrep(filters, '_', '\_'), 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Arial');

xlim([x_min, x_max]);
ylim([y_min_bc, y_max_bc]);

% Set y-ticks with increments based on the data range
yticks(linspace(y_min_bc, y_max_bc, 5)); % Adjust number of ticks based on range
ytickformat('%.0f'); % Ensure no decimal places for ballistic coefficient

% Additional plot formatting
set(gca, 'FontSize', 10, 'LineWidth', 1.5, 'FontName', 'Arial');
xticks(linspace(x_min, x_max, 10));
xtickformat('%.0f'); % Display x-ticks as integers

grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

set(gca, 'Box', 'on');
set(gcf, 'Color', 'w');

% Save the ballistic coefficient comparison plot
saveas(gcf, 'plots/IAC_Cb_Plot_IAC.png');

%% Plot for Predicted Apogee Comparison
figure('Position', [100, 100, 700, 500], "Name", "Predicted Apogee Comparison Plot");

% Initialize plot handles for each filter
plot_handles_apogee = gobjects(1, length(filters));

% Plot apogee estimates for each filter
for i = 1:length(filters)
    if contains(filters(i), 'constant_acceleration')
        marker = constant_accel_markers; % Use circle for constant acceleration models
    else
        marker = constant_cb_markers;    % Use square for constant ballistic coefficient models
    end
    plot_handles_apogee(i) = plot(all_times{i}, all_apogee_est_smooth{i}, [marker, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', colors{i}, ...
        'DisplayName', strrep(filters(i), '_', '\_')); % Display name with LaTeX formatting
    hold on;
end

% Add a horizontal line representing the true apogee
hline = yline(true_apogee, '--k', 'LineWidth', 1.5);

% Add labels, legend, and axis formatting
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Predicted Apogee Error $(m)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');

legend([plot_handles_apogee, hline], [strrep(filters, '_', '\_'), 'True Apogee'], ...
    'Location', 'best', 'FontSize', 10, 'FontName', 'Arial');

set(legend, 'Box', 'on', 'EdgeColor', 'k');

xlim([x_min, x_max]);
ylim([y_min_apogee, y_max_apogee]);

% Set y-ticks with increments based on the data range
yticks(linspace(y_min_apogee, y_max_apogee, 6)); % Adjust number of ticks based on range
ytickformat('%.0f'); % Ensure no decimal places for apogee error

% Additional plot formatting
set(gca, 'FontSize', 10, 'LineWidth', 1.5, 'FontName', 'Arial');
xticks(linspace(x_min, x_max, 10));
xtickformat('%.0f'); % Display x-ticks as integers

grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

set(gca, 'Box', 'on');
set(gcf, 'Color', 'w');

% Save the predicted apogee comparison plot
saveas(gcf, 'plots/IAC_Apogee_Plot_IAC.png');
