% Load data
filename = 'data/real/owen.csv';
data = readtable(filename);
column_headers = data.Properties.VariableNames;
data_struct = struct();

% Store data in a struct
for i = 1:length(column_headers)
    column_name = column_headers{i};
    column_data = data.(column_name);
    data_struct.(column_name) = column_data;
end

% Initialize the timestamps and convert to seconds
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp)) / 1000;

% Define the sample rates (in Hz)
barometer_rate = 1; % 1 Hz
accelerometer_rate = 10; % 10 Hz

% Initialize the Kalman Filter
x = [data_struct.baro_altitude(1); 0; 0]; % Initial state vector [altitude; velocity; acceleration]
P = eye(3); % Initial estimation error covariance

% Observation models
H_b = [1 0 0]; % Observation model for barometer (only altitude)
H_a = [0 0 1]; % Observation model for accelerometer (only acceleration)

% Measurement noise covariance matrices
R_b = 0.1; % Measurement noise covariance for barometer
R_a = 1; % Measurement noise covariance for accelerometer

% Control input (none in this case)
B = [0 0 1]';
u = 0;

% Initialize time and loop counter
t = 0;
barometer_index = 1;
accelerometer_index = 1;
dt = 1 / max(barometer_rate, accelerometer_rate); % Time step

% Initialize arrays for storing estimates and timestamps
timestamps = [];
barometer_estimates = [];
accelerometer_estimates = [];

% Loop over the data
while t <= max(data_struct.timestamp)
    t = t + dt; % Increment time

    % State transition matrix
    A = [1 dt 0.5*dt^2;
         0 1 dt;
         0 0 1];

    % Dynamic process noise covariance matrix
    Q = 1 * [(dt^5)/20, (dt^4)/8, (dt^3)/6;
             (dt^4)/8, (dt^3)/3, (dt^2)/2;
             (dt^3)/6, (dt^2)/2, dt];

    % Predict step
    x = A * x + B * u; % State prediction
    P = A * P * A' + Q; % Covariance prediction

    % Update step for barometer
    if t >= barometer_index / barometer_rate
        z_b = data_struct.baro_altitude(barometer_index); % Measurement
        K = P * H_b' / (H_b * P * H_b' + R_b); % Kalman gain
        x = x + K * (z_b - H_b * x); % State update
        P = (eye(3) - K * H_b) * P; % Covariance update

        % Log timestamp and estimate
        timestamps(end + 1) = t;
        barometer_estimates(end + 1) = x(1);
        accelerometer_estimates(end + 1) = NaN; % Placeholder for accelerometer estimate

        barometer_index = barometer_index + 1; % Increment index
    end

    % Update step for accelerometer
    if t >= accelerometer_index / accelerometer_rate && accelerometer_index <= length(data_struct.imu_accZ)
        z_a = data_struct.imu_accZ(accelerometer_index); % Measurement
        K = P * H_a' / (H_a * P * H_a' + R_a); % Kalman gain
        x = x + K * (z_a - H_a * x); % State update
        P = (eye(3) - K * H_a) * P; % Covariance update

        % Log timestamp and estimate
        timestamps(end + 1) = t;
        barometer_estimates(end) = NaN; % Ensure same length for both estimates
        accelerometer_estimates(end) = x(3);

        accelerometer_index = accelerometer_index + 1; % Increment index
    end
end

% Create table for combined data
combined_data = table(timestamps', barometer_estimates', accelerometer_estimates', ...
                      'VariableNames', {'timestamp', 'barometer_estimate', 'accelerometer_estimate'});

% Write the table to a CSV file
writetable(combined_data, 'combined_data.csv');

disp('Data generated and saved to combined_data.csv.');
