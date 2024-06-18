data = readtable('data/simulated/Regulus/Regulus 100.0Hz.csv');
rng("default")


timestamp = data.Time;

alt_std = 10; % 
acc_std = 10; % 160e-6 for BMI088 


for i = 1:length(data.Z)
    alt(i) = data.Z(i) + alt_std * randn;
    acc(i) = data.Az(i) + acc_std * randn;
end

% Initial state [altitude; velocity; acceleration]
initial_state = [alt(1); 0; 0];
initial_covariance = diag([1, 1, 1]);
process_noise = diag([1e-3, 1e-3, 1e-2]);
% Measurement noise covariance matrix R
measurement_noise = diag([alt_std^2, acc_std^2]);

dt = 0.01;

% Initialize UKF
ukf = UKF(initial_state, initial_covariance, process_noise, measurement_noise, dt);

% Allocate arrays for estimated states
estimated_states = zeros(length(data.Z), 3);
Cbs = zeros(length(data.Z), 1);
% Run UKF
for i = 1:length(data.Z)
    [ukf, ~, ~] = ukf.predict();
    measurement = [alt(i); acc(i)];
    [ukf, updated_state, ~] = ukf.update(measurement);
    estimated_states(i, :) = updated_state';
    
    x = estimated_states(i,:);
    

    rho = get_density(x(1));
    g = get_gravity(x(1));
    Cbs(i) = (rho * x(2)^2) / (2 * (-g - x(3)));

end

% Extract estimates
estimated_altitude = estimated_states(:, 1);
estimated_velocity = estimated_states(:, 2);
estimated_acceleration = estimated_states(:, 3);

% Plot results
figure;
subplot(3, 1, 1);
plot(data.Time, alt, 'r', data.Time, estimated_altitude, 'b');
legend('Measured Altitude', 'Estimated Altitude');
title('Altitude');

subplot(3, 1, 2);
plot(data.Time, estimated_velocity);
title('Velocity');

subplot(3, 1, 3);
plot(data.Time, acc, 'r', data.Time, estimated_acceleration, 'b');
legend('Measured Acceleration', 'Estimated Acceleration');
title('Acceleration');

figure;
hold on;
plot(data.Time, Cbs)
xlim([10,25]);
hold off;

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
