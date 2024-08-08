clear all;
close all;

set(groot,'defaulttextinterpreter','latex');  
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaultLegendInterpreter','latex');

filters = ["UKF_constant_acceleration", "UKF_constant_Cb", "EKF_constant_acceleration", "EKF_constant_Cb"];
time_min = 9;
time_max = 17;
factor = 40;
windowSize = 1;

for i = 1:length(filters)
    filter_name = filters(i);
    data_filename = strcat('data_filtered/', filter_name, '_filtered_data.csv');
    data = csvread(data_filename, 1, 0);
    times = data(:, 1);
    x_est = data(:, 5);
    index = times >= time_min & times <= time_max;
    times = times(index);
    x_est = x_est(index);
    times = times(1:factor:end);
    x_est = x_est(1:factor:end);
    x_est_smooth = movmean(x_est, windowSize);
    all_times{i} = times;
    all_x_est_smooth{i} = x_est_smooth;
end

x_min = min(cellfun(@(x) min(x), all_times));
x_max = max(cellfun(@(x) max(x), all_times));
y_min = min(cellfun(@(y) min(y), all_x_est_smooth));
y_max = max(cellfun(@(y) max(y), all_x_est_smooth));

figure('Position', [100, 100, 700, 500]);

markers = {'o-', 's--', 'd-.', '*:'}; 
colors = {'k', [0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4660 0.6740 0.1880]};

for i = 1:length(filters)
    plot(all_times{i}, all_x_est_smooth{i}, markers{i}, ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'Color', colors{i}, ...
        'DisplayName', strrep(filters(i), '_', '\_')); 
    hold on;
end

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Ballistic Coefficient $(kg/m^2)$', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Arial');

legend('show', 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Arial');

xlim([x_min, x_max]);
ylim([y_min, y_max]);

set(gca, 'FontSize', 10, 'LineWidth', 1.5, 'FontName', 'Arial');
xticks(linspace(x_min, x_max, 10)); 
yticks(linspace(y_min, y_max, 10)); 

xtickformat('%.0f');
ytickformat('%.0f');

grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

set(gca, 'Box', 'on');
set(gcf, 'Color', 'w');

saveas(gcf, 'IAC_Cb_Plot_IAC.png');
