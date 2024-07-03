% Author: Jack Johnston
% Date: 03/07/24

% Code for a constant velocity kalman filter that can be used for state transitions
% for the astra flight computer

% Will be able to reliably detect, launch, apogee and touch-down
% Criteria from burning to coasting, will need to be fixed, and time based
% from launch


%% Load data
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

dt = 0.01; % Time step

% Construct accelerometer data struct
data_struct.imu_acc = [data_struct.imu_accX, data_struct.imu_accY, data_struct.imu_accZ];

% Initialize the timestamps and convert to seconds
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp)) / 1000;
%% Initialise State Machine
StateMachine = stateMachine;

%% Initialise Kalman Filter
alt_init = data_struct.baro_altitude(1);
sigma_alt = 0.5744578867366569; %STD for altitude

% Initial state vector [altitude; vertical velocity]
x_init = [data_struct.baro_altitude(1); 0];

STFilter = kalmanFilterCV(alt_init, sigma_alt, 0.01);

% Initialize arrays for storing estimates
x_est = [];

% Initialize time and loop counter
k = 1;
t = 0;
times = [];

z_b = data_struct.baro_altitude(1);

% Loop until landed or apogee detected or end of data
while t <= max(data_struct.timestamp)
    t = t + dt; % Increment time
    times = [times, t]; % Store current time


    % Predict step
    STFilter = STFilter.predict(dt);

    % Update step if new data is available
    if k <= length(data_struct.timestamp) && t >= data_struct.timestamp(k)
        % Check if it's time for a barometer update
        if data_struct.baro_altitude(k) ~= z_b
            z_b = data_struct.baro_altitude(k); % Barometer measurement
            STFilter = STFilter.update(z_b); % Update state with barometer measurement
        end
        k = k + 1; % Increment measurement index
    end

    % Check for state machine transitions
    StateMachine = StateMachine.check(STFilter.get_states, t);

    % Record the estimated states
    x_est(:, end + 1) = STFilter.get_states;


    

end

run("plotting.m")
