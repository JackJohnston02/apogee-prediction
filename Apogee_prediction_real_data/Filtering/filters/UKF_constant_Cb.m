classdef UKF_constant_Cb
    % Author : Jack Johnston
    % Date : 31/07/24

    % Refactoring and combin\ation of previous code

    % Unscented Kalman filter, for state observation of Cb
    % State estimator for altitude, vertical velocity, vertical
    % acceleration, and vertical ballisitic coefficient

    % Inludes apogee prediction method, which consists of a series of
    % unscented predict steps to obtain and estimate for both apogee and
    % time of apogee as well as the covariance associated with each.

    %%
    properties
        x               % State vector
        P               % State covariance matrix
        Q               % Process noise covariance matrix
        R_acc           % Accelerometer measurement noise variance
        R_bar           % Barometer measurement noise variance
        t_last_update   % Time of last update
        dt_apa          % Timestep of the apogee prediction method
        alpha           % UKF parameter
        beta            % UKF parameter
        kappa           % UKF parameter
        sigma_Q         % Standard deviation of acceleration noise
        sigma_Q_Cb      % Scalr term
    end

    methods
        function obj = UKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
            obj.x = initial_state;
            obj.P = initial_covariance;
            obj.Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, 1e1]);
            obj.R_acc = measurement_sigma_acc^2;
            obj.R_bar = measurement_sigma_bar^2;
            obj.t_last_update = t;
            obj.dt_apa = 0.01;
            obj.alpha = 1e-3;
            obj.beta = 2;
            obj.kappa = 1;
            obj.sigma_Q = sigma_Q; % Initialize obj.sigma_Q
            obj.sigma_Q_Cb = sigma_Q_Cb;% Scaler for Cb Q matrix
        end

        function [obj, predicted_state, predicted_covariance] = predict(obj, t_current)
            % UKF prediction step

            % Generate sigma points
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints(obj.x, obj.P);

            % Calculate dt since last prediction
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;
            % Propagate the sigma points through the processModel
            predicted_sigma_points = obj.processModel(sigma_points, dt);


            predicted_state = predicted_sigma_points * weights_mean;

            % Calculate the covariance of the sigma points adding the
            obj.Q = calculateProcessNoise(obj, dt);
            % process noise as well
            predicted_covariance = obj.calculateCovariance(predicted_sigma_points, predicted_state, weights_cov, obj.Q);

            obj.x = predicted_state;

            obj.P = predicted_covariance;
        end

        function Q = calculateProcessNoise(obj, dt)

            g = obj.get_gravity(obj.x(1));
            if g - obj.x(3) > 0 % Condition for ballistic state
                Q = [1/4*dt^4*obj.sigma_Q^2, 1/2*dt^3*obj.sigma_Q^2, 1/2*dt^2*obj.sigma_Q^2, 0;
                    1/2*dt^3*obj.sigma_Q^2,    dt^2*obj.sigma_Q^2,    dt*obj.sigma_Q^2, 0;
                    1/2*dt^2*obj.sigma_Q^2,      dt*obj.sigma_Q^2,          obj.sigma_Q^2, 0;
                    0, 0, 0, obj.sigma_Q_Cb^2];
            else
                % In constant acceleration mode, set Q_Cb to 0
                Q = [1/4*dt^4*obj.sigma_Q^2, 1/2*dt^3*obj.sigma_Q^2, 1/2*dt^2*obj.sigma_Q^2, 0;
                    1/2*dt^3*obj.sigma_Q^2,    dt^2*obj.sigma_Q^2,    dt*obj.sigma_Q^2, 0;
                    1/2*dt^2*obj.sigma_Q^2,      dt*obj.sigma_Q^2,          obj.sigma_Q^2, 0;
                    0, 0, 0, 0];


            end
        end

        function [apogee, apogee_std] = get_apogee(obj)
            propagated_x = obj.x;
            propagated_P = obj.P;

            % Propagate states forward untill apogee is reached
            while propagated_x(2) > 0 && propagated_x(3) < 0
                [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints(propagated_x, propagated_P);

                % Propagate the sigma points through the processModel
                predicted_sigma_points = obj.processModel(sigma_points, obj.dt_apa);

                predicted_state = predicted_sigma_points * weights_mean;

                predicted_covariance = obj.calculateCovariance(predicted_sigma_points, predicted_state, weights_cov, obj.Q);

                propagated_x = predicted_state;

                propagated_P = predicted_covariance;
            end

            apogee = propagated_x(1);
            apogee_std = sqrt(propagated_P(1,1));

        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            % Pre-processing, acceleromter does not measure gravity
            measurement = measurement + obj.get_gravity(obj.x(1));
            % Update timer for last update
            obj.t_last_update = t_current;

            % UKF Update step for accelerometer
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints(obj.x, obj.P);

            % Only care for states which correspond for measurements
            predicted_measurement_sigma_points = obj.measurementModelAccelerometer(sigma_points);
            predicted_measurement = predicted_measurement_sigma_points * weights_mean;

            innovation_covariance = obj.calculateCovariance(predicted_measurement_sigma_points, predicted_measurement, weights_cov, obj.R_acc);
            cross_covariance = obj.calculateCrossCovariance(sigma_points, obj.x, predicted_measurement_sigma_points, predicted_measurement, weights_cov);
            kalman_gain = cross_covariance / innovation_covariance;
            innovation = measurement - predicted_measurement;
            updated_state = obj.x + kalman_gain * innovation;
            updated_covariance = obj.P - kalman_gain * innovation_covariance * kalman_gain';
            obj.x = updated_state;
            obj.P = updated_covariance;
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


        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)
            % UKF Update step for barometer
            obj.t_last_update = t_current;
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints(obj.x, obj.P);
            predicted_measurement_sigma_points = obj.measurementModelBarometer(sigma_points);
            predicted_measurement = predicted_measurement_sigma_points * weights_mean;
            innovation_covariance = obj.calculateCovariance(predicted_measurement_sigma_points, predicted_measurement, weights_cov, obj.R_bar);
            cross_covariance = obj.calculateCrossCovariance(sigma_points, obj.x, predicted_measurement_sigma_points, predicted_measurement, weights_cov);
            kalman_gain = cross_covariance / innovation_covariance;
            innovation = measurement - predicted_measurement;
            updated_state = obj.x + kalman_gain * innovation;
            updated_covariance = obj.P - kalman_gain * innovation_covariance * kalman_gain';
            obj.x = updated_state;
            obj.P = updated_covariance;
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


        function [sigma_points, weights_mean, weights_cov] = generateSigmaPoints(obj, mean, cov)
            % Generate sigma points

            n = length(mean);
            lambda = obj.alpha^2 * (n + obj.kappa) - n;
            sigma_points = zeros(n, 2*n + 1);
            weights_mean = zeros(2*n + 1, 1);
            weights_cov = zeros(2*n + 1, 1);

            sigma_points(:, 1) = mean;
            weights_mean(1) = lambda / (n + lambda);
            weights_cov(1) = weights_mean(1) + (1 - obj.alpha^2 + obj.beta);

            sqrt_P = chol((n + lambda) * cov)';
            for i = 1:n
                sigma_points(:, i+1) = mean + sqrt_P(:, i);
                sigma_points(:, i+n+1) = mean - sqrt_P(:, i);
                weights_mean(i+1) = 1 / (2 * (n + lambda));
                weights_cov(i+1) = weights_mean(i+1);
                weights_mean(i+n+1) = 1 / (2 * (n + lambda));
                weights_cov(i+n+1) = weights_mean(i+1);
            end
        end


        function sigma_points = processModel(obj, sigma_points, dt)
            % Conditional statement required as we cannot use the constant
            % Cb model when in the burning state, just let Cb track loosely

            for i = 1:size(sigma_points, 2)

                x_s = sigma_points(:, i);

                g = obj.get_gravity(x_s(1));
                rho = obj.get_density(x_s(1));

                if g - x_s(3) > 0 % Model for ballistic state - constant ballistic coefficient
                    x_s(1) = x_s(1) + x_s(2) * dt + 1/2 * x_s(3) * dt^2;
                    x_s(2) = x_s(2) + x_s(3) * dt;
                    x_s(3) = g - (rho * x_s(2)^2) / (2 * x_s(4));
                    x_s(4) = max(x_s(4), 10);  % Clamping to stop Cb going negtative


                else % Model for non-ballistic state - constant acceleration
                    x_s(1) = x_s(1) + x_s(2) * dt + 1/2 * x_s(3) * dt^2;
                    x_s(2) = x_s(2) + x_s(3) * dt;
                    x_s(3) = x_s(3);
                    x_s(4) = x_s(4);%(rho * x_s(2)^2) / (2 * (abs(g - x_s(3))));
                end

                x_new = [x_s(1); x_s(2); x_s(3); x_s(4)];
                sigma_points(:, i) = x_new;
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

