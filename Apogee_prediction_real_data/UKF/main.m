%% TODO
 % Need to impliment the constant_Cb function
 % Need to improve the plotting functions, add switch case statements where
 % neccesary

%% Tidy up
clear all;

%% Select filter type
% UKF_constant_acceleration
% UKF_constant_Cb

filter_type = "UKF_constant_acceleration";

% Load data
filename = 'data/owen.csv';
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
data_struct.imu_acc = [data_struct.imu_accZ]; %Only care about the Z component of the acceleration
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp)) / 1000;


% Initialise logging arrays
x_est = [];
times = [];
apogee_log = [];

% Initial measurements
z_b = data_struct.baro_altitude(1);
z_a = data_struct.imu_acc(1,:);

switch filter_type
    case "UKF_constant_acceleration"
        initial_state = [z_b, 0, z_a, 0]';
        initial_covariance = eye(4);
        process_noise = 1e2*diag([1e-3, 1e-3, 1e-2, 1e-3]);
        measurement_noise_bar = 0.5744578867366569;
        measurement_noise_acc = 0.006942717204787825;
        t = 0;
        dt_apa = 0.01;
        filter = UKF_constant_acceleration(initial_state, initial_covariance, process_noise, measurement_noise_acc, measurement_noise_bar, t, dt_apa);
   
    case "UKF_constant_Cb"
        initial_state = [z_b, 0, z_a, 0]';
        initial_covariance = eye(4);
        process_noise = 1e2*diag([1e-3, 1e-3, 1e-2, 1e-3]);
        measurement_noise_bar = 0.5744578867366569;
        measurement_noise_acc = 0.006942717204787825;
        t = 0;
        dt_apa = 0.01;
        filter = UKF_constant_Cb(initial_state, initial_covariance, process_noise, measurement_noise_acc, measurement_noise_bar, t, dt_apa);
end



dt = 0.01;
k = 1;
for i = 1:(length(data_struct.timestamp))
    t = t + dt; % Increment time

    disp(round((i/length(data_struct.timestamp)*100),2) + "% complete");

    times = [times, t]; % Store current time


    % Predict step
    [filter, predicted_state, predicted_covariance] = filter.predict(t);

    % Update step if new data is available
    if k <= length(data_struct.timestamp) && t >= data_struct.timestamp(k)
        % Check if it's time for a barometer update
        if data_struct.baro_altitude(k) ~= z_b
            z_b = data_struct.baro_altitude(k); % Barometer measurement
            [filter, updated_state, updated_covariance] = filter.updateBarometer(z_b, t); % Update state with barometer measurement
        end

        % Check if it's time for an accelerometer update
        if data_struct.imu_acc(k,:) ~= z_a
            z_a = data_struct.imu_acc(k); % Accelerometer measurement
            [filter, updated_state, updated_covariance] = filter.updateAccelerometer(z_a, t); % Update state with accelerometer measurement
        end

        k = k + 1; % Increment measurement index
    end

    % Record the estimated states
    x_est(:, end + 1) = filter.x;

    [apogee, apogee_covariance] = filter.get_apogee();

    apogee_log = [apogee_log,[t, apogee, apogee_covariance]'];

end

% Save the Cb data for plotting
exportMatrix = [times', x_est(:,:)'];
writematrix(exportMatrix',"data_filtered/UKF_filtered_data.csv")


run("plotting.m")


