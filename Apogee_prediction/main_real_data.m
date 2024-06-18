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
data_struct.timestamp = (data_struct.timestamp - min(data_struct.timestamp))/1000;

APA = APA_Single_Particle(0.01);


%% Initialize the Kalman Filter
% Initial state vector [altitude; velocity; acceleration]
x = [data_struct.baro_altitude(1); 0; 0];

% Estimation error covariance matrix
P = eye(3); % Initial estimation error covariance

% Observation model
H = [1 0 0; 0 0 1]; % Only altitude and acceleration are observed

% Static process noise covariance matrix
Q = [1 0 0;
     0 1 0;
     0 0 1];

% Measurement noise covariance matrix
R = diag([0.1, 1]);

% Initialize arrays for storing estimates
x_est = [];
densities = [];
Cbs = [];

% Control input (none in this case)
B = [0 0 1]';
u = 0;
uout = [];

% Initialize time and loop counter
k = 1;
t = 0;
times = [];
dt = 0.01; % Time step

%% Loop until landed or apogee detected or end of data
for i = 1:length(data_struct.timestamp)
    t = t + dt; % Increment time
    times = [times, t]; % Store current time

    % Dynamic process noise covariance matrix
    Q = 1*[(dt^5)/20, (dt^4)/8, (dt^3)/6;
           (dt^4)/8, (dt^3)/3, (dt^2)/2;
           (dt^3)/6, (dt^2)/2, dt];

    % State transition matrix
    A = [1 dt 0.5*dt^2;
         0 1 dt;
         0 0 1];

    % Predict step
    x = A*x + B*u; % State prediction
    P = A*P*A' + Q; % Covariance prediction

    % Update step if new data is available
    if k <= length(data_struct.timestamp) && t >= data_struct.timestamp(k)
        z = [data_struct.baro_altitude(k); data_struct.imu_accZ(k)]; % Measurement vector
        K = P*H' / (H*P*H' + R); % Compute Kalman gain
        x = x + K * (z - H*x); % State update
        P = (eye(3) - K*H) * P; % Covariance update
        k = k + 1; % Increment measurement index
    end

    % Record the estimated states
    x_est(:, end + 1) = x;

    % Calculate ballistic coefficient
    rho = get_density(x(1));
    g = get_gravity(x(1));
    Cbs(end+1) = (rho * x(2)^2) / (2 * (-x(3)));
    
    if x(3) < 0 
        [APA, predicted_apogee] = APA.getApogee(t,x);
    end
end

%% Post analysis
figure;
subplot(1,1,1);
hold on;
plot(APA.log_apogee(1,:), APA.log_apogee(2,:), 'b', 'LineWidth', 2);
plot(data_struct.timestamp, data_struct.baro_altitude)
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Estimated Altitude vs Actual Altitude');
legend('Estimated Altitude', 'Actual Altitude');

% Plotting states
figure;
subplot(3,1,1);
plot(times, x_est(1,:), 'b', 'LineWidth', 2);
hold on;
plot(data_struct.timestamp, data_struct.baro_altitude, 'r', 'LineWidth', 2);
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
plot(times, x_est(3,:), 'b', 'LineWidth', 2);
hold on;
plot(data_struct.timestamp, data_struct.imu_accZ, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Estimated Acceleration vs Actual Acceleration');
legend('Estimated Acceleration', 'Actual Acceleration');

% Plotting the ballistic coefficient
figure;

xlimmin = 9.5;
xlimmax = 15.5;

plot(times, Cbs, 'r', 'LineWidth', 2);
hold on;
mmaResult = forward_reverse_moving_average(Cbs, 100);
plot(times, mmaResult, 'b', 'LineWidth', 2);
xlim([xlimmin, xlimmax]);
ylim([0, 2000]);

xlabel('Time (s)');
ylabel('Ballistic Coefficient');
title('Ballistic Coefficient Over Time');
legend('Estimated Ballistic Coefficient', 'Moving Average');

%% Function for density
function rho = get_density(h)
    % Returns atmospheric density as a function of altitude
    % Accurate up to 11km
    % https://en.wikipedia.org/wiki/Density_of_air

    p_0 = 101325; % Standard sea level atmospheric pressure
    M = 0.0289652; % molar mass of dry air
    R = 8.31445; % ideal gas constant
    T_0 = 288.15; % Standard sea level temperature
    L = 0.0065; % temperature lapse rate
    g = get_gravity(h);

    rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1); % -g used as g is -ve by default
end

%% Function for gravity
function g = get_gravity(h)
    % Returns gravity as a function of altitude
    % Approximates the Earth's gravity assumes a perfect sphere
    
    g_0 = -9.80665; % Standard gravity
    R_e = 6371000; % Earth radius

    g = g_0 * (R_e / (R_e + h))^2;
end

%% Moving Average Function
function result = forward_reverse_moving_average(data, window_size)
    % Apply forward moving average
    forward_avg = movmean(data, window_size);
    
    % Apply reverse moving average
    reversed_data = fliplr(data);
    reverse_avg = movmean(reversed_data, window_size);
    reverse_avg = fliplr(reverse_avg);
    
    % Average both results
    result = (forward_avg + reverse_avg) / 2;
end
