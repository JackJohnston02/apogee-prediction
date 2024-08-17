classdef CKF_constant_Cb
    % Author : Jack Johnston (Modified to CKF)
    % Date : 31/07/24 (Modified)

    % Cubature Kalman filter, for state observation of Cb
    % State estimator for altitude, vertical velocity, vertical
    % acceleration, and vertical ballistic coefficient

    properties
        x               % State vector
        P               % State covariance matrix
        Q               % Process noise covariance matrix
        R_acc           % Accelerometer measurement noise variance
        R_bar           % Barometer measurement noise variance
        t_last_update   % Time of last update
        dt_apa          % Timestep of the apogee prediction method
        sigma_Q         % Standard deviation of acceleration noise
        sigma_Q_Cb      % Scalar term for Q
    end

    methods
        function obj = CKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
            obj.x = initial_state;
            obj.P = initial_covariance;
            obj.Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, 1e1]);
            obj.R_acc = measurement_sigma_acc^2;
            obj.R_bar = measurement_sigma_bar^2;
            obj.t_last_update = t;
            obj.dt_apa = 0.01;
            obj.sigma_Q = sigma_Q;
            obj.sigma_Q_Cb = sigma_Q_Cb;
        end

        function [obj, predicted_state, predicted_covariance] = predict(obj, t_current)
            % CKF prediction step

            % Calculate dt since last prediction
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;

            % Generate cubature points
            cubature_points = obj.generateCubaturePoints(obj.x, obj.P);

            % Propagate the cubature points through the processModel
            predicted_cubature_points = obj.processModel(cubature_points, dt);

            predicted_state = mean(predicted_cubature_points, 2);

            % Calculate the covariance of the cubature points adding the process noise
            obj.Q = obj.calculateProcessNoise(dt);
            predicted_covariance = obj.calculateCovariance(predicted_cubature_points, predicted_state, obj.Q);

            obj.x = predicted_state;
            obj.P = predicted_covariance;
        end

        function cubature_points = processModel(obj, cubature_points, dt)
            % Conditional statement required as we cannot use the constant
            % Cb model when in the burning state, just let Cb track loosely

            % Number of cubature points
            num_points = size(cubature_points, 2);

            % Iterate over each cubature point
            for i = 1:num_points
                x_s = cubature_points(:, i);

                g = obj.get_gravity(x_s(1));
                rho = obj.get_density(x_s(1));

                if g - x_s(3) > 0 % Model for ballistic state - constant ballistic coefficient
                    x_s(1) = x_s(1) + x_s(2) * dt + 1/2 * x_s(3) * dt^2;
                    x_s(2) = x_s(2) + x_s(3) * dt;
                    x_s(3) = g - (rho * x_s(2)^2) / (2 * x_s(4));
                    x_s(4) = x_s(4);

                else % Model for non-ballistic state - constant acceleration
                    x_s(1) = x_s(1) + x_s(2) * dt + 1/2 * x_s(3) * dt^2;
                    x_s(2) = x_s(2) + x_s(3) * dt;
                    x_s(3) = x_s(3);
                    x_s(4) = x_s(4);%(rho * x_s(2)^2) / (2 * (abs(g - x_s(3))));
                end

                % Update cubature point with new state
                cubature_points(:, i) = x_s;

            end
        end



        function Q = calculateProcessNoise(obj, dt)

            g = obj.get_gravity(obj.x(1));
            if g - obj.x(3) > 0 % Model for ballistic state
                % Calculate the process noise covariance matrix Q
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

            while propagated_x(2) > 0 && propagated_x(3) < 0
                cubature_points = obj.generateCubaturePoints(propagated_x, propagated_P);

                % Propagate the cubature points through the processModel
                predicted_cubature_points = obj.processModel(cubature_points, obj.dt_apa);

                predicted_state = mean(predicted_cubature_points, 2);

                predicted_covariance = obj.calculateCovariance(predicted_cubature_points, predicted_state, obj.Q);

                propagated_x = predicted_state;
                propagated_P = predicted_covariance;
            end

            apogee = propagated_x(1);
            apogee_std = sqrt(propagated_P(1,1));
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            % Pre-processing, accelerometer does not measure gravity
            measurement = measurement + obj.get_gravity(obj.x(1));
            obj.t_last_update = t_current;

            % CKF Update step for accelerometer
            cubature_points = obj.generateCubaturePoints(obj.x, obj.P);

            % Measurement prediction
            predicted_measurement_points = obj.measurementModelAccelerometer(cubature_points);
            predicted_measurement = mean(predicted_measurement_points, 2);

            innovation_covariance = obj.calculateCovariance(predicted_measurement_points, predicted_measurement, obj.R_acc);
            cross_covariance = obj.calculateCrossCovariance(cubature_points, obj.x, predicted_measurement_points, predicted_measurement);
            kalman_gain = cross_covariance / innovation_covariance;
            innovation = measurement - predicted_measurement;
            updated_state = obj.x + kalman_gain * innovation;
            updated_covariance = obj.P - kalman_gain * innovation_covariance * kalman_gain';
            obj.x = updated_state;

            obj.P = updated_covariance;
        end

        function predicted_measurement_points = measurementModelAccelerometer(obj, cubature_points)
            % Define the measurement model for the accelerometer
            predicted_measurement_points = cubature_points(3, :);
        end

        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)
            % CKF Update step for barometer
            obj.t_last_update = t_current;
            cubature_points = obj.generateCubaturePoints(obj.x, obj.P);
            predicted_measurement_points = obj.measurementModelBarometer(cubature_points);
            predicted_measurement = mean(predicted_measurement_points, 2);
            innovation_covariance = obj.calculateCovariance(predicted_measurement_points, predicted_measurement, obj.R_bar);
            cross_covariance = obj.calculateCrossCovariance(cubature_points, obj.x, predicted_measurement_points, predicted_measurement);
            kalman_gain = cross_covariance / innovation_covariance;
            innovation = measurement - predicted_measurement;
            updated_state = obj.x + kalman_gain * innovation;
            updated_covariance = obj.P - kalman_gain * innovation_covariance * kalman_gain';
            obj.x = updated_state;
            obj.P = updated_covariance;
        end

        function predicted_measurement_points = measurementModelBarometer(obj, cubature_points)
            % Define the measurement model for the barometer
            predicted_measurement_points = cubature_points(1, :);
        end

        function cubature_points = generateCubaturePoints(~, mean, cov)
            % Generate cubature points
            n = length(mean);
            cubature_points = zeros(n, 2*n);

            sqrt_P = chol(cov)';
            for i = 1:n
                cubature_points(:, i) = mean + sqrt(n) * sqrt_P(:, i);
                cubature_points(:, i+n) = mean - sqrt(n) * sqrt_P(:, i);
            end
        end

        function covariance = calculateCovariance(~, points, mean, noise_cov)
            % Calculate covariance matrix
            n = size(points, 1);
            covariance = zeros(n, n);
            for i = 1:size(points, 2)
                diff = points(:, i) - mean;
                covariance = covariance + (diff * diff');
            end
            covariance = covariance / size(points, 2);
            covariance = covariance + noise_cov;
        end

        function cross_covariance = calculateCrossCovariance(~, state_points, state_mean, measurement_points, measurement_mean)
            % Calculate cross-covariance matrix
            n = size(state_points, 1);
            cross_covariance = zeros(n, size(measurement_points, 1));
            for i = 1:size(state_points, 2)
                state_diff = state_points(:, i) - state_mean;
                measurement_diff = measurement_points(:, i) - measurement_mean;
                cross_covariance = cross_covariance + (state_diff * measurement_diff');
            end
            cross_covariance = cross_covariance / size(state_points, 2);
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
