
plot_dynamics = true;
plot_control_output = true;
animation_airbrake_position = true; 


%% Dynamics output
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

%% Control output
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
    grid on;  % add a grid

    subplot(4, 1, 2);  % create a subplot
    plot(t, airbrake_position_log, 'LineWidth', 2);
    title('Airbrake Position vs Time');
    xlabel('Time (s)');
    ylabel('Degrees');
    grid on;  % add a grid

    subplot(4, 1, 3);  % create a subplot
    plot(t, error_log, 'LineWidth', 2);
    title('Error vs Time');
    xlabel('Time (s)');
    ylabel('Error');
    grid on;  % add a grid

    subplot(4, 1, 4);  % create a subplot
    plot(t, error_log, 'LineWidth', 2);
    title('Zoomed Error vs Time');
    xlabel('Time (s)');
    ylabel('Error');
    ylim([-5,5])
    xlim([0,t(end)])
    grid on;  % add a grid
end



%% Animate the airbrakes
if animation_airbrake_position == true
    figure;
    ax = axes;
    clear figure;
    
    % Create a rectangle
    originalVertices = [0 0; 0.1 0; 0.1 -1.5; 0 -1.5];
    rectangle = patch('Vertices', originalVertices, 'Faces', [1 2 3 4], 'FaceColor', 'b');
    
    % Set the axes limits
    axis([-2 2 -2 2]);
    axis square;
    
    viscircles([0 0], 0.02, 'Color', 'r');
    
    % Create a text object for displaying time
    timeText = text(0, 1.5, '', 'HorizontalAlignment', 'center','FontSize', 14);
    posText = text(0, 1.25, '', 'HorizontalAlignment', 'center','FontSize', 14);
    
    % Animate the rectangle
    for i = 1:length(t)-1
        % Start a timer
        tic;
        
        % Convert the angle from degrees to radians
        theta = deg2rad(airbrake_position_log(i));
        
        % Create the rotation matrix
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        % Apply the rotation matrix to the rectangle vertices
        rectangle.Vertices = (R * originalVertices')';
        
        % Update the plot
        drawnow;
        
        timeText.String = ['Time: ', sprintf('%.2f', t(i))];
        posText.String = ['Angle: ', sprintf('%.2f', airbrake_position_log(i))];
        
        % Compute the time it took to execute the above commands
        elapsedTime = toc;
        
        % Pause for the time difference between frames, adjusted for the elapsed time
        pauseTime = t(i+1) - t(i) - elapsedTime;
        if pauseTime > 0
            pause(pauseTime);
        end
    end
end