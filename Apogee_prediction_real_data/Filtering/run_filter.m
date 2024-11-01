% Load data
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

% Normalize timestamp
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp)) / 1000;

% Tare barometer data
data_struct.baro_altitude = (data_struct.baro_altitude) - data_struct.baro_altitude(1); 

% Remove repeated duplicate values
fields = fieldnames(data_struct);

% Remove duplicates based on timestamp
[unique_timestamps, unique_indices] = unique(data_struct.timestamp, 'stable');

for i = 1:length(fields)
    field_name = fields{i};
    data_struct.(field_name) = data_struct.(field_name)(unique_indices);
end

% Crop data
data_struct = crop_data_struct(data_struct, 0, 30);

% Initialise logging arrays
x_est = [];
times = [];
apogee_log = [];

% Initial measurements
z_b = data_struct.baro_altitude(1);
z_a = data_struct.imu_accZ(1);

initial_state = [z_b, 0, 0, 1000]'; %% Cb_0 can be set "arbitrarily"
initial_covariance = 0.01 * eye(4);

t = 0;

%% Initialise filter object
switch filter_type
    case "UKF_constant_acceleration"
        filter = UKF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Acceleration UKF";

    case "UKF_constant_Cb"
        filter = UKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Ballistic Coefficient UKF";

    case "EKF_constant_acceleration"
        filter = EKF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Acceleration EKF";

    case "EKF_constant_Cb"
        filter = EKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Ballistic Coefficient EKF";

    case "CKF_constant_acceleration"
        filter = CKF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Acceleration CKF";

    case "CKF_constant_Cb"
        filter = CKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Ballistic Coefficient CKF";

    case "SIRPF_constant_acceleration"
        filter = SIRPF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t);
        filter_name = "Constant Acceleration Particle Filter";
    otherwise
        % Throw error if invalid filter is selected
        error("Invalid choice of filter")
end

dt = 0.01;
k = 1;

figure_waitbar = waitbar(0,'Please wait...');
while t < apogee_time

    frac_complete = t/max(data_struct.timestamp);
    % Waitbar
    waitbar(frac_complete,figure_waitbar,"Running " + filter_name);


    t = t + dt; % Increment time
    times = [times, t]; % Store current time

    % Predict step
    tic;
    [filter, predicted_state, predicted_covariance] = filter.predict(t);
    filterTimes.(filter_type).timeUpdateTimes(end+1) = toc;


    % Update step if new data is available
    if k <= length(data_struct.timestamp) && t >= data_struct.timestamp(k)
        tic;   
        % Check if it's time for a barometer update
        if data_struct.baro_altitude(k) ~= z_b
            z_b = data_struct.baro_altitude(k); % Barometer measurement
            [filter, updated_state, updated_covariance] = filter.updateBarometer(z_b, t); % Update state with barometer measurement
        end

        % Check if it's time for an accelerometer update
        if any(data_struct.imu_accZ(k) ~= z_a)
            z_a = data_struct.imu_accZ(k); % Accelerometer measurement
            [filter, updated_state, updated_covariance] = filter.updateAccelerometer(z_a, t); % Update state with accelerometer measurement
        end
    
        filterTimes.(filter_type).measurementUpdateTimes(end+1) = toc;
        k = k + 1; % Increment measurement index
    end


    tic;
    [apogee, apogee_std] = filter.get_apogee();
    filterTimes.(filter_type).apogeePredictionTimes(end+1) = toc;

    % Record the estimated states and estimated apogee
    x_est(:, end + 1) = [filter.x; apogee; apogee_std];

    apogee_log = [apogee_log,[t, apogee, apogee_std]'];


    % Exit statement
    if x_est(2,:) < -1
        break
    end

end
waitbar(frac_complete,figure_waitbar,"Plotting " + filter_name);
close(figure_waitbar);

%% Exporting the estimated states
exportMatrix = [times', x_est(:,:)'];
writematrix(exportMatrix,"data_filtered/" + filter_type +"_filtered_data.csv")

run("plotting.m")

function cropped_struct = crop_data_struct(data_struct, start_time, end_time)
% Find the indices where the timestamp is within the specified range
valid_indices = data_struct.timestamp >= start_time & data_struct.timestamp <= end_time;
cropped_struct = struct();

% Iterate over each field in the struct and crop the data
field_names = fieldnames(data_struct);
for i = 1:length(field_names)
    field = field_names{i};
    cropped_struct.(field) = data_struct.(field)(valid_indices);
end
end
