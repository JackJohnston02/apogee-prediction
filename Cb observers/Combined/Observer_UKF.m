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
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints();
            predicted_sigma_points = obj.processModel(sigma_points);
            predicted_state = predicted_sigma_points * weights_mean;
            predicted_covariance = obj.calculateCovariance(predicted_sigma_points, predicted_state, weights_cov, obj.Q);
            obj.state = predicted_state;
            obj.P = predicted_covariance;
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement)
            measurement = measurement(3) - 9.81; % Pre-processing
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
            n = length(obj.state);
            lambda = obj.alpha^2 * (n + obj.kappa) - n;
            sigma_points = zeros(n, 2 * n + 1);
            weights_mean = zeros(2 * n + 1, 1);
            weights_cov = zeros(2 * n + 1, 1);

            sigma_points(:, 1) = obj.state;
            weights_mean(1) = lambda / (n + lambda);
            weights_cov(1) = weights_mean(1) + (1 - obj.alpha^2 + obj.beta);

            sqrt_P = sqrtm((n + lambda) * obj.P);
            for i = 1:n
                sigma_points(:, i + 1) = obj.state + sqrt_P(:, i);
                sigma_points(:, i + n + 1) = obj.state - sqrt_P(:, i);
                weights_mean(i + 1) = 1 / (2 * (n + lambda));
                weights_cov(i + 1) = 1 / (2 * (n + lambda));
                weights_mean(i + n + 1) = 1 / (2 * (n + lambda));
                weights_cov(i + n + 1) = 1 / (2 * (n + lambda));
            end
        end

        function predicted_sigma_points = processModel(obj, sigma_points)
            n = size(sigma_points, 1);
            predicted_sigma_points = sigma_points;
            for i = 1:size(sigma_points, 2)
                predicted_sigma_points(:, i) = sigma_points(:, i) + obj.dt * [sigma_points(2, i); ...
                    -obj.get_density(sigma_points(1, i)) * sigma_points(2, i)^2 / (2 * sigma_points(3, i)) + obj.get_gravity(sigma_points(1, i)); ...
                    0];
            end
        end

        function predicted_measurement_sigma_points = measurementModelAccelerometer(obj, sigma_points)
            predicted_measurement_sigma_points = sigma_points(3, :);
        end

        function predicted_measurement_sigma_points = measurementModelBarometer(obj, sigma_points)
            predicted_measurement_sigma_points = sigma_points(1, :);
        end

        function cov = calculateCovariance(obj, sigma_points, mean, weights_cov, noise_cov)
            n = size(sigma_points, 1);
            cov = zeros(n, n);
            for i = 1:size(sigma_points, 2)
                diff = sigma_points(:, i) - mean;
                cov = cov + weights_cov(i) * (diff * diff');
            end
            cov = cov + noise_cov;
        end

        function cross_cov = calculateCrossCovariance(obj, sigma_points, state_mean, measurement_sigma_points, measurement_mean, weights_cov)
            n = length(state_mean);
            m = size(measurement_sigma_points, 1);
            cross_cov = zeros(n, m);
            for i = 1:size(sigma_points, 2)
                state_diff = sigma_points(:, i) - state_mean;
                measurement_diff = measurement_sigma_points(:, i) - measurement_mean;
                cross_cov = cross_cov + weights_cov(i) * (state_diff * measurement_diff');
            end
        end

        function rho = get_density(obj, h)
            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % molar mass of dry air
            R = 8.31445; % ideal gas constant
            T_0 = 288.15; % Standard sea level temperature
            L = 0.0065; % temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M) / (R * T_0) * (1 - (L * h) / (T_0)) ^ ((-g * M) / (R * L) - 1); % -g used as g is -ve by default
        end

        function g = get_gravity(obj, h)
            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h)) ^ 2;
        end
    end
end
