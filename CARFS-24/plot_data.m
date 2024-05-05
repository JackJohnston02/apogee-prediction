
plot_dynamics = true;
plot_control_output = true;


if plot_dynamics == true
    dynamics_fig = figure;
    % Make plots appear fulscreen
    set(dynamics_fig, 'WindowState', 'maximized');

    set(dynamics_fig, 'Name', 'Rocket Dynamics', 'NumberTitle', 'off');

    % Plot altitude
    subplot(3, 1, 1);  % create a subplot
    plot(t, Rocket.x(:,1), 'LineWidth', 2);
    title('Altitude vs Time');
    xlabel('Time (s)');
    ylabel('Altitude (m)');
    grid on;  % add a grid

    % Plot position
    subplot(3, 1, 2);  % create a subplot
    plot(t, Rocket.x(:,2), 'LineWidth', 2);
    title('Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity (m/2)');
    grid on;  % add a grid

    % Plot velocity
    subplot(3, 1, 3);  % create a subplot
    plot(t, Rocket.x(:,3), 'LineWidth', 2);
    title('Acceleration vs Time');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    grid on;  % add a grid
end

if plot_control_output == true
    
    control_output_fig = figure;
    % Make plots appear fulscreen
    set(control_output_fig, 'WindowState', 'maximized');
    set(control_output_fig, 'Name', 'Rocket Dynamics', 'NumberTitle', 'off');

    subplot(4, 1, 1);  % create a subplot
    plot(t, airbrake_velocity_log, 'LineWidth', 2);
    title('Airbrake Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Degrees/s');

    subplot(4, 1, 2);  % create a subplot
    plot(t, airbrake_position_log, 'LineWidth', 2);
    title('Airbrake Position vs Time');
    xlabel('Time (s)');
    ylabel('Degrees');

    subplot(4, 1, 3);  % create a subplot
    plot(t, error_log, 'LineWidth', 2);
    title('Error vs Time');
    xlabel('Time (s)');
    ylabel('Error');
    
    subplot(4, 1, 4);  % create a subplot
    plot(t, error_log, 'LineWidth', 2);
    title('Zoomed Error vs Time');
    xlabel('Time (s)');
    ylabel('Error');
    ylim([-5,5])
    xlim([0,t(end)])
end
