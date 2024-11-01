% main.m

% Load data
filename = 'data/real/owen.csv';
export_filename = "owen";
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
x_init = [data_struct.baro_altitude(1); 1; 1500];

% Estimation error covariance matrix
P_init = eye(3);

% Static process noise covariance matrix
Q_s = 1e2; % scaling parameter for process noise

% Measurement noise covariance matrices
R_b = 0.5744578867366569; % Measurement noise covariance for barometer
R_a = 1e5;%0.006942717204787825; % Measurement noise covariance for accelerometer

% Initialize the timestamps and convert to seconds
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp)) / 1000;

% Create EKF object
ekf = Observer_EKF(x_init, P_init, Q_s, R_a, R_b, dt);

% Initialize arrays for storing estimates
x_est = [];
p_est = [];
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
while t < 20
    t = t + dt; % Increment time
    times = [times, t]; % Store current time

    % Predict step
    [ekf, predicted_state, predicted_covariance] = ekf.predict(t);

        % Update step if new data is available
        if k <= length(data_struct.timestamp) && t >= data_struct.timestamp(k)
            % Check if it's time for a barometer update
            if data_struct.baro_altitude(k) ~= z_b
    
                z_b = data_struct.baro_altitude(k); % Barometer measurement
                [ekf, updated_state, updated_covariance] = ekf.updateBarometer(z_b); % Update state with barometer measurement
            end
    
            % Check if it's time for an accelerometer update
            if data_struct.imu_acc(k,:) ~= z_a
               z_a = data_struct.imu_acc(k,:); % Accelerometer measurement
               [ekf, updated_state, updated_covariance] = ekf.updateAccelerometer(z_a(3)); % Update state with accelerometer measurement
            end
            k = k + 1; % Increment measurement index
        end
    % Record the estimated states
    disp(ekf.x)
    x_est(:, end + 1) = [ekf.x]';
    % Record the estimated uncertainty in each of the states
    p_est(:, end+1) = [ekf.P(1,1), ekf.P(2,2), ekf.P(3,3)]';
end

% Save the Cb data for plotting
exportMatrix = [times', x_est(3,:)'];
writematrix(exportMatrix',"data_filtered/" + export_filename + "_EKF_filtered.csv") 

% Plot results
plotting(times, x_est, p_est, data_struct);
