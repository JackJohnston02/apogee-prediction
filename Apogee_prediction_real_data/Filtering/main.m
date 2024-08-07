%% TODO
% Add EKF 
    % CCb
    % Ca
% Add Cubature
    % CCb
    % Ca

% Add iterative script for all filters
% Add comparison plotting for all filters
  

%% Tidy up
clear all;
close all; % Uncomment if comparing plots
pause(1);
%% Select filter type
% UKF_constant_acceleration
% UKF_constant_Cb

filter_type = "UKF_constant_Cb";

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

data_struct = crop_data_struct(data_struct, 0, 18);

% Initialise logging arrays
x_est = [];
times = [];
apogee_log = [];

% Initial measurements
z_b = data_struct.baro_altitude(1);
z_a = data_struct.imu_acc(1,:);


initial_state = [z_b, 0, z_a, 1300]';
initial_covariance = eye(4);
sigma_Q = 1e1;
sigma_Q_Cb = 1;
measurement_noise_bar = 0.5744578867366569;
measurement_noise_acc = 0.006942717204787825;
t = 0;

%% Initialise filter object
switch filter_type
    case "UKF_constant_acceleration"
        filter = UKF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_noise_acc, measurement_noise_bar, t);
   
    case "UKF_constant_Cb"
       filter = UKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_noise_acc, measurement_noise_bar, t);
            
    otherwise
        % Throw error if invalid filter is selected
        error("Invalid choice of filter")
end



dt = 0.01;
k = 1;

while t < max(data_struct.timestamp)
    disp(round((t/max(data_struct.timestamp)*100),2) + "% complete at time:" + t);
    
    t = t + dt; % Increment time
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


function cropped_struct = crop_data_struct(data_struct, start_time, end_time)
    % Find the indices where the timestamp is within the specified range
    valid_indices = data_struct.timestamp >= start_time & data_struct.timestamp <= end_time;
    
    % Initialize the cropped struct
    cropped_struct = struct();

    % Iterate over each field in the struct and crop the data
    field_names = fieldnames(data_struct);
    for i = 1:length(field_names)
        field = field_names{i};
        cropped_struct.(field) = data_struct.(field)(valid_indices);
    end
end



