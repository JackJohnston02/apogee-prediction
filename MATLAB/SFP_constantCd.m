% Linear Kalman Filter for state estimation
clear all
data = readtable('flight_data.csv');

CROP = 2000;
alt = data.baro_altitude(1:CROP);
acc = (data.imu_accZ(1:CROP) - mean(data.imu_accZ(1:300))); %"Tared" ACC

timestamp = (data.timestamp - data.timestamp(1))/1000;
timestamp = timestamp(1:CROP);

% initial states
x = [alt(1); 0; 0]; % [altitude; velocity; acceleration]

% Estimation error covariance
P = eye(3);

% System dynamics
H = [1 0 0; 0 0 1]; % Observation model

Q = 0.0001 *[1 0 0 ; 
             0 1 0; 
             0 0 1]; % Process noise covariance %0.001 * eye(3) 
R = diag([var(alt(1:300)), var(acc(1:300))]); % Measurement noise covariance

% Initialize arrays to store the state estimates
x_est = zeros(3, length(alt));
x_est(:, 1) = x;

launch_detected = false;
apogee_detected = false;
motor_burntout = false;

launch_time = 0;
motor_burntout = 0;
apogee_time = 0;
apogee_altitude = 0;

predicted_apogee_time = 0;
predicted_apogee_altitude = 0;
predicted_apogee_times = [0];
predicted_apogee_altitudes = [0];
lower_bounds = [0];
upper_bounds = [0];
prediction_times = [0];




for k = 2:length(alt)
    dt = timestamp(k) - timestamp(k-1);

    % Update the system dynamics

    A = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1]; % State transition
    % Predict state and estimation error covariance
    x = A*x;
    P = A*P*A' + Q;

    % Compute Kalman gain
    K = P*H' / (H*P*H' + R);

    % Update state and estimation error covariance
    z = [alt(k); acc(k)]; % Measurement vector
    x = x + K * (z - H*x);
    P = (eye(3) - K*H) * P;
    
    % record the states
    x_est(:, k) = x;

    % Check for launch 
    if ~launch_detected && x_est(2, k) > 10 % Assuming launch when velocity > 10
        launch_detected = true;
        launch_time = timestamp(k);
    end
    
    % Check motor burnout
    if launch_detected && ~motor_burntout && x_est(3, k) < 0 % Assuming motor burnout when acceleration < 0
        motor_burntout = true;
        burnout_time = timestamp(k);
    end

    % Predict apogee after 1 second after motor burn-out and before apogee is detected
    if motor_burntout && timestamp(k) >  0.1 + burnout_time && ~apogee_detected
    
        [lower_bound, predicted_apogee_altitude, upper_bound] =  FP_complexModel(x, P, timestamp(k), dt);

        predicted_apogee_altitudes = [predicted_apogee_altitudes, predicted_apogee_altitude];
        lower_bounds = [lower_bounds, lower_bound];
        upper_bounds = [upper_bounds, upper_bound];

        prediction_times = [prediction_times, timestamp(k)];
    end

    % Check apogee
    if motor_burntout && ~apogee_detected && x_est(2, k) < 0
        apogee_detected = true;
        apogee_time = timestamp(k);
        apogee_altitude = x_est(1, k);
    end

    

    %if k == length(alt) %INSTANT PLOTTING
    if mod(k, 1000) == 0  %REALish TIME
        clf;  % Clear  figure
        tiledlayout(1,1);

        % Altitude estimates and measured data
        nexttile;
        plot(timestamp, alt, 'y');
        hold on;
        plot(timestamp(1:k), x_est(1, 1:k), 'b');
        
        %Plot lower, mean and upper of apogee predictions
        
        scatter(prediction_times, lower_bounds,2, 'r'); 
        scatter(prediction_times, predicted_apogee_altitudes,3, 'g'); 
        scatter(prediction_times, upper_bounds,2, 'b'); 

        title('Altitude Estimates and Measured Data');
        xlabel('Time (s)');
        ylabel('Altitude (m)');
        legend('Measured Altitude', 'Estimated Altitude', 'Propagated Altitude');

      
        
        xline(launch_time, 'g:', 'DisplayName', 'Launch Time');

        xline(burnout_time, 'g:', 'DisplayName', 'Motor Burnout Time');
        
        xline(apogee_time, 'g', 'DisplayName', 'Actual Apogee Time');
        yline(apogee_altitude, 'g', 'DisplayName', 'Actual Apogee Altitude');

        hold off;

        drawnow;  % Update plot
    end
    
end

% Final apogee detection information
disp(['Apogee detected at time: ', num2str(apogee_time), ' seconds']);
disp(['Apogee altitude: ', num2str(apogee_altitude), ' meters']);



%propagationError(apogee_altitude,predicted_apogee_altitudes(:), prediction_times(:))
