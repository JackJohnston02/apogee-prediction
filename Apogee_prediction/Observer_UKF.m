classdef Observer_UKF
    properties
        state          % State vector
        P              % State covariance matrix
        Q              % Process noise covariance matrix
        R_acc          % Accelerometer measurement noise covariance matrix
        R_bar          % Barometer measurement noise covariance matrix
        dt             % Time step
        alpha          % UKF parameter
        beta           % UKF parameter
        kappa          % UKF parameter
    end

    methods
        function obj = Observer_UKF(initial_state, initial_covariance, process_noise, measurement_noise_acc, measurement_noise_bar, dt)
            obj.state = initial_state;
            obj.P = initial_covariance;
            obj.Q = process_noise;
            obj.R_acc = measurement_noise_acc;
            obj.R_bar = measurement_noise_bar;
            obj.dt = dt;
            obj.alpha = 1e-3;
            obj.beta = 2;
            obj.kappa = 0;
        end

        function [obj, predicted_state, predicted_covariance] = predict(obj)
            % UKF Predict step
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints();
            predicted_sigma_points = obj.processModel(sigma_points);
            predicted_state = predicted_sigma_points * weights_mean;
            predicted_covariance = obj.calculateCovariance(predicted_sigma_points, predicted_state, weights_cov, obj.Q);
            obj.state = predicted_state;
            obj.P = predicted_covariance;
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement)
            % UKF Update step for accelerometer
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints();
            predicted_measurement_sigma_points = obj.measurementModelAccelerometer(sigma_points);
            predicted_measurement = predicted_measurement_sigma_points * weights_mean;
            innovation_covariance = obj.calculateCovariance(predicted_measurement_sigma_points, predicted_measurement, weights_cov, obj.R_acc);
            cross_covariance = obj.calculateCrossCovariance(sigma_points, obj.state, predicted_measurement_sigma_points, predicted_measurement, weights_cov);
            kalman_gain = cross_covariance / innovation_covariance;
            innovation = measurement - predicted_measurement;
            updated_state = obj.state + kalman_gain * innovation;
            updated_covariance = obj.P - kalman_gain * innovation_covariance * kalman_gain';
            obj.state = updated_state;
            obj.P = updated_covariance;
        end

        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement)
            % UKF Update step for barometer
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints();
            predicted_measurement_sigma_points = obj.measurementModelBarometer(sigma_points);
            predicted_measurement = predicted_measurement_sigma_points * weights_mean;
            innovation_covariance = obj.calculateCovariance(predicted_measurement_sigma_points, predicted_measurement, weights_cov, obj.R_bar);
            cross_covariance = obj.calculateCrossCovariance(sigma_points, obj.state, predicted_measurement_sigma_points, predicted_measurement, weights_cov);
            kalman_gain = cross_covariance / innovation_covariance;
            innovation = measurement - predicted_measurement;
            updated_state = obj.state + kalman_gain * innovation;
            updated_covariance = obj.P - kalman_gain * innovation_covariance * kalman_gain';
            obj.state = updated_state;
            obj.P = updated_covariance;
        end

        function [sigma_points, weights_mean, weights_cov] = generateSigmaPoints(obj)
            % Generate sigma points
            n = length(obj.state);
            lambda = obj.alpha^2 * (n + obj.kappa) - n;
            sigma_points = zeros(n, 2*n + 1);
            weights_mean = zeros(2*n + 1, 1);
            weights_cov = zeros(2*n + 1, 1);

            sigma_points(:, 1) = obj.state;
            weights_mean(1) = lambda / (n + lambda);
            weights_cov(1) = weights_mean(1) + (1 - obj.alpha^2 + obj.beta);

            sqrt_P = chol((n + lambda) * obj.P)';
            for i = 1:n
                sigma_points(:, i+1) = obj.state + sqrt_P(:, i);
                sigma_points(:, i+n+1) = obj.state - sqrt_P(:, i);
                weights_mean(i+1) = 1 / (2 * (n + lambda));
                weights_cov(i+1) = weights_mean(i+1);
                weights_mean(i+n+1) = 1 / (2 * (n + lambda));
                weights_cov(i+n+1) = weights_mean(i+1);
            end
        end

        function sigma_points = processModel(obj, sigma_points)
            % Define the process model
            % This should be replaced with the actual process model
            dt = obj.dt;
            for i = 1:size(sigma_points, 2)
                x = sigma_points(:, i);
                rho = obj.get_density(x(1));
                x(1) = x(1) + x(2) * dt + 1/2 * x(3) * dt^2;
                x(2) = x(2) + x(3) * dt;
                

                if x(1) > 400 % Transition to free coast
                    % We require an incrimental change in x(3) due to dt,
                    % for proper state tracking         
                    x(3) = -9.81 - (rho * (x(2)^2))/(2 * x(4));
                    x(4) = x(4);%rho * x(2)^2 / (2 * (abs(x(3)) - 9.81));

                else 
                    x(3) = x(3);
                    x(4) = (rho * x(2)^2)/(2 * (abs(x(3)) - 9.81));
                end
                
                x_new = [x(1); x(2); x(3); x(4)];
                sigma_points(:, i) = x_new;
            end
        end

        function predicted_measurement_sigma_points = measurementModelAccelerometer(obj, sigma_points)
            % Define the measurement model for the accelerometer
            predicted_measurement_sigma_points = zeros(1, size(sigma_points, 2));
            for i = 1:size(sigma_points, 2)
                x = sigma_points(:, i);
                z = x(3);  % Assuming the accelerometer measures acceleration
                predicted_measurement_sigma_points(:, i) = z;
            end
        end

        function predicted_measurement_sigma_points = measurementModelBarometer(obj, sigma_points)
            % Define the measurement model for the barometer
            predicted_measurement_sigma_points = zeros(1, size(sigma_points, 2));
            for i = 1:size(sigma_points, 2)
                x = sigma_points(:, i);
                z = x(1);  % Assuming the barometer measures altitude
                predicted_measurement_sigma_points(:, i) = z;
            end
        end

        function covariance = calculateCovariance(obj, sigma_points, mean, weights, noise_cov)
            % Calculate covariance matrix
            n = size(sigma_points, 1);
            covariance = zeros(n, n);
            for i = 1:size(sigma_points, 2)
                diff = sigma_points(:, i) - mean;
                covariance = covariance + weights(i) * (diff * diff');
            end
            covariance = covariance + noise_cov;
        end

        function cross_covariance = calculateCrossCovariance(obj, sigma_points, state_mean, predicted_measurement_sigma_points, predicted_measurement, weights_cov)
            % Calculate cross-covariance matrix
            n = size(sigma_points, 1);
            cross_covariance = zeros(n, size(predicted_measurement_sigma_points, 1));
            for i = 1:size(sigma_points, 2)
                state_diff = sigma_points(:, i) - state_mean;
                measurement_diff = predicted_measurement_sigma_points(:, i) - predicted_measurement;
                cross_covariance = cross_covariance + weights_cov(i) * (state_diff * measurement_diff');
            end
        end

        function rho = get_density(obj, h)
            % Returns atmospheric density as a function of altitude
            % Accurate up to 11km
            % https://en.wikipedia.org/wiki/Density_of_air

            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % molar mass of dry air
            R = 8.31445; % ideal gas constant
            T_0 = 288.15; % Standard sea level temperature
            L = 0.0065; % temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1); % -g used as g is -ve by default
        end
        function g = get_gravity(obj, h)
            % Returns gravity as a function of altitude
            % Approximates the Earth's gravity assumes a perfect sphere

            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h))^2;
        end


    end
end
