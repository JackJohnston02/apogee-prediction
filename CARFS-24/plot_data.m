
plot_dynamics = true;
plot_control_output = true;
animation_airbrake_position = true; 
Serial_stream = Serial_streamer("COM8", 9600, false);



%% Dynamics output
if plot_dynamics == true
    dynamics_fig = figure;
    % Make plots appear fulscreen
    set(dynamics_fig, 'WindowState', 'maximized');

    set(dynamics_fig, 'Name', 'Rocket Dynamics', 'NumberTitle', 'off');

    % Plot altitude
    subplot(3, 1, 1);  % create a subplot
    hold on;
    scatter(t, error_log + targetApogee, 2, "red"); %Predicted apogee at time t
    plot(t, Rocket.x(:,1), 'LineWidth', 2, "Color",[0 0 1]); %Altitude at time t
    hold off; 
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

    subplot(5, 1, 1);  % create a subplot
    plot(t, airbrake_velocity_log*60, 'LineWidth', 2);
    yline(Rocket.Airbrake.maxVelocity*60, "red");
    yline(Rocket.Airbrake.minVelocity*60, "red");
    title('Stepper Motor Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity (RPM)');
    xlim([0,t(end)])
    grid on;  % add a grid

    subplot(5, 1, 2);  % create a subplot
    plot(t, (-airbrake_position_log + Rocket.Airbrake.maxP) * 1000, 'LineWidth', 2);
    yline((-Rocket.Airbrake.maxP + Rocket.Airbrake.maxP) * 1000, "red");
    yline((-Rocket.Airbrake.minP + Rocket.Airbrake.maxP) * 1000, "red");
    title('Crucifix position vs Time');
    xlabel('Time (s)');
    ylabel('P (m)');
    xlim([0,t(end)])
    grid on;  % add a grid

    subplot(5, 1, 3);  % create a subplot
    plot(t, airbrake_angle_log, 'LineWidth', 2);
    yline(0, "red");
    yline(Rocket.Airbrake.maxAngle, "red");
    title('Airbrake Angle vs Time');
    xlabel('Time (s)');
    ylabel('Degrees');
    xlim([0,t(end)])
    grid on;  % add a grid

    subplot(5, 1, 4);  % create a subplot
    plot(t, error_log, 'LineWidth', 2);
    title('Error vs Time');
    xlabel('Time (s)');
    ylabel('Error');
    xlim([0,t(end)])
    grid on;  % add a grid

    subplot(5, 1, 5);  % create a subplot
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
    
    % Create a rectangle with non-zero dimensions
    originalVertices = [0 0; 5 0; 5 -110; 0 -110];
    rectangleC = patch('Vertices', [-36, 75; 36, 75; 35.75, 0; -35.75, 0], 'Faces', [1 2 3 4], 'FaceColor', 'B');
    rectangleL = patch('Vertices', originalVertices, 'Faces', [1 2 3 4], 'FaceColor', 'r');
    rectangleR = patch('Vertices', originalVertices, 'Faces', [1 2 3 4], 'FaceColor', 'r');
    
    % Set the axes limits to ensure the rectangle is visible
    axis([-120 120 -120 120]);
    axis square;
    
    circleL = viscircles([-35.75 0], 1, 'Color', 'r');  % Adjusted the circle radius for visibility
    circleR = viscircles([35.75 0], 1, 'Color', 'r');  % Adjusted the circle radius for visibility
    % Create a text object for displaying time
    timeText = text(0, 110, '', 'HorizontalAlignment', 'center','FontSize', 14);
    posText = text(0, 100, '', 'HorizontalAlignment', 'center','FontSize', 14);
    
    % Animate the rectangle
    for i = 1:length(t)-1
        % send velocity data
        Serial_stream.startStreaming(airbrake_velocity_log(i) * 60)


        % Start a timer
        tic;
        
        % Convert the angle from degrees to radians
        theta = deg2rad(airbrake_angle_log(i));
        
        % Create the rotation matrix
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        % Apply the rotation matrix to the rectangle vertices
        rotatedVertices = (R * originalVertices')';
        
        % Mirror the rectangle around the vertical axis (y-axis)
        mirroredVertices = rotatedVertices;
        mirroredVertices(:,1) = -mirroredVertices(:,1);  % Negate the x-coordinates
        
        % Update the vertices of the rectangles
        rectangleR.Vertices = rotatedVertices;
        rectangleL.Vertices = mirroredVertices;
        
        % Translate both flaps, away from central axis
        rectangleR.Vertices = rectangleR.Vertices + [35.75,0];
        rectangleL.Vertices = rectangleL.Vertices + [-35.75,0];
        % Update the plot
        drawnow;
        
        timeText.String = ['Time: ', sprintf('%.2f', t(i))];
        posText.String = ['Angle: ', sprintf('%.2f', airbrake_angle_log(i))];
        
        % Compute the time it took to execute the above commands
        elapsedTime = toc;
        
        % Pause for the time difference between frames, adjusted for the elapsed time
        pauseTime = t(i+1) - t(i) - elapsedTime;
        if pauseTime > 0
            pause(pauseTime);
        end
    end
    Serial_stream.delete()
end

