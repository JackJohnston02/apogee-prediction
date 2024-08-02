% Load dat
filename = 'data/real/owen.csv';
data = readtable(filename);
data = data(1:end, :);
column_headers = data.Properties.VariableNames;
data_struct = struct();

% Store data in a struct
for i = 1:length(column_headers)
    column_name = column_headers{i};
    column_data = data.(column_name);
    data_struct.(column_name) = column_data;
end

dt = 0.01; % Time step

% Subtract gravity

data_struct.imu_acc = [data_struct.imu_accX, data_struct.imu_accY, data_struct.imu_accZ];


% Initial state vector [altitude; velocity; acceleration; ballistic coefficient]
x_init = [data_struct.baro_altitude(1); 0; 0; 0];

% Estimation error covariance matrix
P_init = eye(4);

% Static process noise covariance matrix (to be updated dynamically)

% For ballisitic coefficient observer
Q = 1e-2* [(dt^5)/20, (dt^4)/8, (dt^3)/6, 0;
        (dt^4)/8, (dt^3)/3, (dt^2)/2, 1e-32;
       (dt^3)/6, (dt^2)/2, dt, 1e-16;
        0, 0, 1e-16, 1e-15];


% Measurement noise covariance matrices
R_b = 0.54; % Measurement noise covariance for barometer
R_a = 1e-3; % Measurement noise covariance for accelerometer

% Initialize the timestamps and convert to seconds
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp)) / 1000;

% Create UKF object
ukf = Observer_UKF(x_init, P_init, Q, R_a, R_b, dt);
apa = APA_Single_Particle(dt);

% Initialize arrays for storing estimates
x_est = [];
densities = [];
Cbs = [];

% Initialize time and loop counter
k = 1;
t = 0;
times = [];

z_b = data_struct.baro_altitude(1);
z_a = data_struct.imu_acc(1,:);

rho = 1.225;
apogees = [];
% Loop until landed or apogee detected or end of data
for i = 1:length(data_struct.timestamp)
    t = t + dt; % Increment time
    times = [times, t]; % Store current time


    % Predict step
    [ukf, predicted_state, predicted_covariance] = ukf.predict();

    % Update step if new data is available
    if k <= length(data_struct.timestamp) && t >= data_struct.timestamp(k)
        % Check if it's time for a barometer update
        if data_struct.baro_altitude(k) ~= z_b
            z_b = data_struct.baro_altitude(k); % Barometer measurement
            [ukf, updated_state, updated_covariance] = ukf.updateBarometer(z_b); % Update state with barometer measurement
        end

        % Check if it's time for an accelerometer update
        if data_struct.imu_acc(k,:) ~= z_a
            z_a = data_struct.imu_acc(k,:); % Accelerometer measurement
            [ukf, updated_state, updated_covariance] = ukf.updateAccelerometer(z_a); % Update state with accelerometer measurement
        end

        k = k + 1; % Increment measurement index
    end

    % Record the estimated states
    x_est(:, end + 1) = ukf.state;
    
    if ukf.state(1) > 400 && ukf.state(2) > 0
        [apa,apogee] = apa.getApogee(t, ukf.state);
    end

end

% Save the Cb data for plotting
exportMatrix = [times', x_est(:,:)'];
writematrix(exportMatrix',"data_filtered/kalman_filtered_data.csv") 


% Plotting states
figure;
subplot(3,1,1);
plot(times, x_est(1,:), 'b', 'LineWidth', 0.5);
hold on;
scatter(data_struct.timestamp, data_struct.baro_altitude, 4, "LineWidth", 0.1);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Estimated Altitude vs Actual Altitude');
legend('Estimated Altitude', 'Actual Altitude');

subplot(3,1,2);
plot(times, x_est(2,:), 'b', 'LineWidth', 2);
hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Velocity');
legend('Estimated Velocity');

subplot(3,1,3);
plot(times, x_est(3,:), 'b', 'LineWidth', 0.5);
hold on;
scatter(data_struct.timestamp, data_struct.imu_accZ, 4, "LineWidth", 0.1);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Estimated Acceleration vs Actual Acceleration');
legend('Estimated Acceleration', 'Actual Acceleration');

% Plotting the ballistic coefficient
figure;
hold on;

scatter(apa.log_Cb(1,:), apa.log_Cb(2,:), 4, "red", "LineWidth", 1)
plot(times, x_est(4,:), 'b', 'LineWidth', 0.5);
yline(1510);
grid("on");
yline(1500);
ylim([-2000, 2000]);
xlim([9.14,18]);
%ylim([0,1500]);
xlabel('Time (s)');
ylabel('Ballistic Coefficient');
title('Ballistic Coefficient Over Time');
legend('Calculated', 'Observed');
hold off


figure;
hold on;
title('Predicted Apogee Over Time');
plot(apa.log_apogee(1,:), apa.log_apogee(2,:))
plot(times, x_est(1,:))
grid on;
grid minor;
hold off




