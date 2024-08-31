% Set default text interpreter to LaTeX for better formatting in plots
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');


% General settings
time_min = 9;
time_max = 25;
factor = 100;
windowSize = 1;

% Ballistic Coefficient Plot settings
y_max_bc = 1600;
y_min_bc = 100;

% Percentage error plot settings
percentage_zoom = 5;

% Apogee error plot settings
zoom_apogee_plot = 25;

% List of filters, including CKF, for comparison
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", ...
           "EKF_constant_acceleration", "EKF_constant_Cb", ...
           "CKF_constant_acceleration", "CKF_constant_Cb"];

% Define symbols for different model types
constant_accel_markers = 'o'; % Symbol for constant acceleration models
constant_cb_markers = 's';    % Symbol for constant ballistic coefficient models

% Define colorblind-friendly colors corresponding to each filter type
ukf_color = "#627db8";
ekf_color = "#45ae8d";
ckf_color = "#fb6025";

% Initialize cells to store time and smoothed estimates for each filter
all_times = cell(1, length(filters));
all_x_est_smooth = cell(1, length(filters));
all_apogee_est_smooth = cell(1, length(filters));
all_percentage_errors = cell(1, length(filters));

% Initialize variables
true_apogee = NaN; % Use NaN to signify that it's not set initially

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

    % Compute percentage apogee errors
    if filter_name == "UKF_constant_Cb"
        true_apogee = apogee_est_smooth(end);
    end
    
    % Only compute percentage errors if true_apogee is defined
    if ~isnan(true_apogee)
        apogee_errors = (apogee_est_smooth - true_apogee) / true_apogee * 100;
        all_percentage_errors{i} = apogee_errors;
    else
        all_percentage_errors{i} = NaN(size(apogee_est_smooth)); % Placeholder for undefined errors
    end
end

% Determine plot limits based on the processed data
x_min = min(cellfun(@(x) min(x), all_times));
x_max = max(cellfun(@(x) max(x), all_times));

y_min_apogee = round(true_apogee, -1) - zoom_apogee_plot; % Min y-limit for apogee plot
y_max_apogee = round(true_apogee, -1) + zoom_apogee_plot; % Max y-limit for apogee plot
y_min_percentage = -percentage_zoom;
y_max_percentage = percentage_zoom;

%% Plot for Ballistic Coefficient Comparison
figure('Position', [100, 100, 700, 500], "Name", "Ballistic Coefficient Comparison Plot");

% Initialize plot handles for each filter
plot_handles_bc = gobjects(1, length(filters));

% Plot ballistic coefficient estimates for each filter
for i = 1:length(filters)
    filter_name = filters(i);
    
    % Determine marker based on model type
    if contains(filter_name, 'constant_acceleration')
        marker = constant_accel_markers; % Use circle for constant acceleration models
    else
        marker = constant_cb_markers;    % Use square for constant ballistic coefficient models
    end
    
    % Determine color based on filter type
    if contains(filter_name, 'UKF')
        color = ukf_color;
    elseif contains(filter_name, 'EKF')
        color = ekf_color;
    elseif contains(filter_name, 'CKF')
        color = ckf_color;
    else
        color = [0 0 0]; % Default color if none of the types match
    end
    
    plot_handles_bc(i) = plot(all_times{i}, all_x_est_smooth{i}, [marker, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', color, ...
        'DisplayName', strrep(filter_name, '_', '\_')); % Display name with LaTeX formatting
    hold on;
end

% Add labels, legend, and axis formatting
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Ballistic Coefficient $(kg/m^2)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(plot_handles_bc, strrep(filters, '_', '\_'), 'Location', 'southwest', 'FontSize', 10, 'FontName', 'Arial');

xlim([x_min, x_max]);
ylim([y_min_bc, y_max_bc]);

% Set y-ticks with increments based on the data range
yticks(linspace(y_min_bc, y_max_bc, 6)); % Adjust number of ticks based on range
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
saveas(gcf, 'plots/Comparison_Cb_' + flightname + '.png');

%% Plot for Predicted Apogee Comparison
figure('Position', [100, 100, 700, 500], "Name", "Predicted Apogee Comparison Plot");

% Initialize plot handles for each filter
plot_handles_apogee = gobjects(1, length(filters));

% Plot apogee estimates for each filter
for i = 1:length(filters)
    filter_name = filters(i);
    
    % Determine marker based on model type
    if contains(filter_name, 'constant_acceleration')
        marker = constant_accel_markers; % Use circle for constant acceleration models
    else
        marker = constant_cb_markers;    % Use square for constant ballistic coefficient models
    end
    
    % Determine color based on filter type
    if contains(filter_name, 'UKF')
        color = ukf_color;
    elseif contains(filter_name, 'EKF')
        color = ekf_color;
    elseif contains(filter_name, 'CKF')
        color = ckf_color;
    else
        color = [0 0 0]; % Default color if none of the types match
    end
    
    plot_handles_apogee(i) = plot(all_times{i}, all_apogee_est_smooth{i}, [marker, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', color, ...
        'DisplayName', strrep(filter_name, '_', '\_')); % Display name with LaTeX formatting
    hold on;
end

% Add a horizontal line representing the true apogee
hline = yline(true_apogee, '--k', 'LineWidth', 1.5);

% Add labels, legend, and axis formatting
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Predicted Apogee Error $(m)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');

legend([plot_handles_apogee, hline], [strrep(filters, '_', '\_'), 'True Apogee'], ...
    'Location', 'best', 'FontSize', 10, 'FontName', 'Arial');

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
saveas(gcf, 'plots/Comparison_Predicted_apogee_' + flightname + '.png');

%% Plot for Percentage Apogee Errors
figure('Position', [100, 100, 700, 500], "Name", "Percentage Apogee Error Plot");

% Initialize plot handles for each filter
plot_handles_percentage = gobjects(1, length(filters));

% Plot percentage apogee errors for each filter
for i = 1:length(filters)
    filter_name = filters(i);
    
    % Determine marker based on model type
    if contains(filter_name, 'constant_acceleration')
        marker = constant_accel_markers; % Use circle for constant acceleration models
    else
        marker = constant_cb_markers;    % Use square for constant ballistic coefficient models
    end
    
    % Determine color based on filter type
    if contains(filter_name, 'UKF')
        color = ukf_color;
    elseif contains(filter_name, 'EKF')
        color = ekf_color;
    elseif contains(filter_name, 'CKF')
        color = ckf_color;
    else
        color = [0 0 0]; % Default color if none of the types match
    end
    
    plot_handles_percentage(i) = plot(all_times{i}, all_percentage_errors{i}, [marker, '-'], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', color, ...
        'DisplayName', strrep(filter_name, '_', '\_')); % Display name with LaTeX formatting
    hold on;
end

% Add labels, legend, and axis formatting
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Percentage Apogee Error $(\%)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(plot_handles_percentage, strrep(filters, '_', '\_'), 'Location', 'best', 'FontSize', 10, 'FontName', 'Arial');

xlim([x_min, x_max]);
ylim([y_min_percentage, y_max_percentage]);

% Set y-ticks with increments based on the data range
yticks(linspace(y_min_percentage, y_max_percentage, 5)); % Adjust number of ticks based on range
ytickformat('%.1f'); % Display y-ticks with one decimal place

% Additional plot formatting
set(gca, 'FontSize', 10, 'LineWidth', 1.5, 'FontName', 'Arial');
xticks(linspace(x_min, x_max, 10));
xtickformat('%.0f'); % Display x-ticks as integers

grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

set(gca, 'Box', 'on');
set(gcf, 'Color', 'w');

% Save the percentage apogee error plot
saveas(gcf, 'plots/Comparison_Percentage_Apogee_Error_Plot_' + flightname + '.png');
