% Calculate standard deviations for each state
std_altitude = sqrt(p_est(1, :));
std_velocity = sqrt(p_est(2, :));
std_acceleration = sqrt(p_est(3, :));

% Plotting states with STD bounds
figure;

% Plot Altitude with STD bounds
subplot(3, 1, 1);
hold on;
fill([times, fliplr(times)], [x_est(1,:) + 3*std_altitude, fliplr(x_est(1,:) - 3*std_altitude)], [0.9 0.9 0.9], 'EdgeColor', 'none');
fill([times, fliplr(times)], [x_est(1,:) + 2*std_altitude, fliplr(x_est(1,:) - 2*std_altitude)], [0.7 0.7 0.7], 'EdgeColor', 'none');
fill([times, fliplr(times)], [x_est(1,:) + std_altitude, fliplr(x_est(1,:) - std_altitude)], [0.5 0.5 0.5], 'EdgeColor', 'none');
plot(times, x_est(1,:), 'b', 'LineWidth', 0.5);
scatter(data_struct.timestamp, data_struct.baro_altitude, 4, "LineWidth", 0.1);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Estimated Altitude vs Actual Altitude');
legend('3 STD', '2 STD', '1 STD', 'Estimated Altitude', 'Actual Altitude');

% Plot Velocity with STD bounds
subplot(3, 1, 2);
hold on;
fill([times, fliplr(times)], [x_est(2,:) + 3*std_velocity, fliplr(x_est(2,:) - 3*std_velocity)], [0.9 0.9 0.9], 'EdgeColor', 'none');
fill([times, fliplr(times)], [x_est(2,:) + 2*std_velocity, fliplr(x_est(2,:) - 2*std_velocity)], [0.7 0.7 0.7], 'EdgeColor', 'none');
fill([times, fliplr(times)], [x_est(2,:) + std_velocity, fliplr(x_est(2,:) - std_velocity)], [0.5 0.5 0.5], 'EdgeColor', 'none');
plot(times, x_est(2,:), 'b', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Velocity');
legend('3 STD', '2 STD', '1 STD', 'Estimated Velocity');

% Plot Acceleration with STD bounds
subplot(3, 1, 3);
hold on;
fill([times, fliplr(times)], [x_est(3,:) + 3*std_acceleration, fliplr(x_est(3,:) - 3*std_acceleration)], [0.9 0.9 0.9], 'EdgeColor', 'none');
fill([times, fliplr(times)], [x_est(3,:) + 2*std_acceleration, fliplr(x_est(3,:) - 2*std_acceleration)], [0.7 0.7 0.7], 'EdgeColor', 'none');
fill([times, fliplr(times)], [x_est(3,:) + std_acceleration, fliplr(x_est(3,:) - std_acceleration)], [0.5 0.5 0.5], 'EdgeColor', 'none');
plot(times, x_est(3,:), 'b', 'LineWidth', 0.5);
scatter(data_struct.timestamp, data_struct.imu_accZ, 4, "LineWidth", 0.1);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Estimated Acceleration vs Actual Acceleration');
legend('3 STD', '2 STD', '1 STD', 'Estimated Acceleration', 'Actual Acceleration');

hold off;


% Assuming you have times, x_est, and p_est
Cb = x_est(4, :);
Cb_var = p_est(4, :);  % Assuming these are variance values
Cb_std = sqrt(Cb_var); % Calculate standard deviation

% Display some values for debugging
disp('Variance values:');
disp(Cb_var(1:10)); % Display the first 10 variance values for checking
disp('Standard deviation values:');
disp(Cb_std(1:10)); % Display the first 10 standard deviation values for checking

% Calculate bounds for 1, 2, and 3 standard deviations
upper1 = Cb + Cb_std;
lower1 = Cb - Cb_std;
upper2 = Cb + 2 * Cb_std;
lower2 = Cb - 2 * Cb_std;
upper3 = Cb + 3 * Cb_std;
lower3 = Cb - 3 * Cb_std;

figure;
hold on;

% Plot 3 STD bounds
fill([times, fliplr(times)], [upper3, fliplr(lower3)], [0.9 0.9 0.9], 'EdgeColor', 'none');
% Plot 2 STD bounds
fill([times, fliplr(times)], [upper2, fliplr(lower2)], [0.7 0.7 0.7], 'EdgeColor', 'none');
% Plot 1 STD bounds
fill([times, fliplr(times)], [upper1, fliplr(lower1)], [0.5 0.5 0.5], 'EdgeColor', 'none');

% Plot the central line
plot(times, Cb, 'b', 'LineWidth', 0.5);

grid("on");
ylim([0, 2000]);
xlim([9.14, 17.73]);
xlabel('Time (s)');
ylabel('Ballistic Coefficient');
title('Ballistic Coefficient Over Time');
legend('3 STD', '2 STD', '1 STD', 'Calculated', 'Location', 'best');
hold off;

