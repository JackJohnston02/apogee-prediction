% Plotting states
figure;
tiledlayout(4,1);

% Altitude plot
ax1 = nexttile;
hold on;
plot(times, x_est(1,:), 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5); % Blue
scatter(apogee_log(1,:), apogee_log(2,:), 36, 'x', 'MarkerEdgeColor', [0.8500, 0.3250, 0.0980]); % Orange
scatter(data_struct.timestamp, data_struct.baro_altitude, 10, 'x', 'MarkerEdgeColor', [0.9290, 0.6940, 0.1250]); % Yellow
yline(max(x_est(1,:)), '--k', 'LineWidth', 1.5); % Black dashed line
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Altitude (m)', 'FontWeight', 'bold');
title('Estimated Altitude vs Actual Altitude', 'FontWeight', 'bold');
legend('Estimated Altitude', 'Apogee', 'Barometer Altitude', 'Location', 'best');
ylim([200, 900]);
grid on;
hold off;

% Velocity plot
ax2 = nexttile;
plot(times, x_est(2,:), 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5); % Blue
hold on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Velocity (m/s)', 'FontWeight', 'bold');
title('Estimated Velocity', 'FontWeight', 'bold');
legend('Estimated Velocity', 'Location', 'best');
grid on;
hold off;

% Acceleration plot
ax3 = nexttile;
plot(times, x_est(3,:), 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5); % Blue
hold on;
scatter(data_struct.timestamp, data_struct.imu_accZ - 9.81, 10, 'x', 'MarkerEdgeColor', [0.9290, 0.6940, 0.1250]); % Orange
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Acceleration (m/s^2)', 'FontWeight', 'bold');
title('Estimated Acceleration vs Actual Acceleration', 'FontWeight', 'bold');
legend('Estimated Acceleration', 'IMU Acceleration', 'Location', 'best');
yline(-9.81);
grid on;
hold off;

% Ballistic Coefficient plot
ax4 = nexttile;
plot(times, x_est(4,:), 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5); % Blue
hold on;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Ballistic Coefficient', 'FontWeight', 'bold');
title('Estimated Ballistic Coefficient', 'FontWeight', 'bold');
ylim([1400, 1500]);
grid on;
hold off;

% Link x-axes
linkaxes([ax1, ax2, ax3, ax4], 'x');

% Calculate bounds for 1, 2, and 3 standard deviations
apogee_error = apogee_log(2,:) - max(x_est(1,:));
apogee_std = sqrt(apogee_log(3,:));
upper1 = apogee_error + apogee_std;
lower1 = apogee_error - apogee_std;
upper2 = apogee_error + 2 * apogee_std;
lower2 = apogee_error - 2 * apogee_std;
upper3 = apogee_error + 3 * apogee_std;
lower3 = apogee_error - 3 * apogee_std;

% Create the figure
figure;
hold on;
fill([apogee_log(1,:), fliplr(apogee_log(1,:))], [upper3, fliplr(lower3)], [0.9, 0.9, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
fill([apogee_log(1,:), fliplr(apogee_log(1,:))], [upper2, fliplr(lower2)], [0.7, 0.7, 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
fill([apogee_log(1,:), fliplr(apogee_log(1,:))], [upper1, fliplr(lower1)], [0.5, 0.5, 0.5], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
plot(apogee_log(1,:), apogee_error, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5); % Blue

% Add labels and title
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Apogee Prediction Error', 'FontWeight', 'bold');
title('Predicted Apogee Error with Standard Deviations', 'FontWeight', 'bold');
legend('3 STD', '2 STD', '1 STD', 'Predicted Apogee Error', 'Location', 'best');
grid on;
grid minor;
yline(0, '--k', 'LineWidth', 1.5); % Black dashed line
ylim([-100, 100]);
xlim([9, 17.65]);
hold off;

