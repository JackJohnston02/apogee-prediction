% plotting.m

function plotting(times, x_est, p_est, data_struct)
    % Calculate standard deviations for each state
    std_altitude = sqrt(p_est(1, :));
    std_velocity = sqrt(p_est(2, :));
    std_acceleration = sqrt(p_est(3, :));

    % Plotting states with STD bounds
    figure;

    % Plot Altitude with STD bounds
    subplot(2, 1, 1);
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
    subplot(2, 1, 2);
    hold on;
    fill([times, fliplr(times)], [x_est(2,:) + 3*std_velocity, fliplr(x_est(2,:) - 3*std_velocity)], [0.9 0.9 0.9], 'EdgeColor', 'none');
    fill([times, fliplr(times)], [x_est(2,:) + 2*std_velocity, fliplr(x_est(2,:) - 2*std_velocity)], [0.7 0.7 0.7], 'EdgeColor', 'none');
    fill([times, fliplr(times)], [x_est(2,:) + std_velocity, fliplr(x_est(2,:) - std_velocity)], [0.5 0.5 0.5], 'EdgeColor', 'none');
    plot(times, x_est(2,:), 'b', 'LineWidth', 0.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Estimated Velocity');
    legend('3 STD', '2 STD', '1 STD', 'Estimated Velocity');


    % Ballistic coefficient plotting
    Cb = x_est(3, :);
    Cb_var = p_est(3, :);  % Assuming these are variance values
    Cb_std = sqrt(Cb_var); % Calculate standard deviation

    % Calculate bounds for 1, 2, and 3 standard deviations
    upper1 = Cb + Cb_std;
    lower1 = Cb - Cb_std;
    upper2 = Cb + 2 * Cb_std;
    lower2 = Cb - 2 * Cb_std;
    upper3 = Cb + 3 * Cb_std;
    lower3 = Cb - 3 * Cb_std;

    % Create the figure
    figure;

    % Plot the shaded areas for the standard deviations
    hold on;
    fill([times, fliplr(times)], [upper3, fliplr(lower3)], [0.9 0.9 0.9], 'EdgeColor', 'none');
    fill([times, fliplr(times)], [upper2, fliplr(lower2)], [0.7 0.7 0.7], 'EdgeColor', 'none');
    fill([times, fliplr(times)], [upper1, fliplr(lower1)], [0.5 0.5 0.5], 'EdgeColor', 'none');

    % Plot the actual ballistic coefficient
    plot(times, Cb, 'b', 'LineWidth', 1.5);

    % Add labels and title
    xlabel('Time (s)');
    ylabel('Ballistic Coefficient');
    title('Estimated Ballistic Coefficient with Standard Deviations');
    legend('3 STD', '2 STD', '1 STD', 'Ballistic Coefficient');
    ylim([0, 2000]);
    xlim([9.14, 17.73]);

    hold off;
end
