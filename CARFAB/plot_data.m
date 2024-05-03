
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



