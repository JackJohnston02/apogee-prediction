classdef UKF
    properties
        state          % State vector
        P              % State covariance matrix
        Q              % Process noise covariance matrix
        R              % Measurement noise covariance matrix
        dt             % Time step
        alpha          % UKF parameter
        beta           % UKF parameter
        kappa          % UKF parameter
    end

    methods
        function obj = UKF(initial_state, initial_covariance, process_noise, measurement_noise, dt)
            obj.state = initial_state;
            obj.P = initial_covariance;
            obj.Q = process_noise;
            obj.R = measurement_noise;
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

        function [obj, updated_state, updated_covariance] = update(obj, measurement)
            % UKF Update step
            [sigma_points, weights_mean, weights_cov] = obj.generateSigmaPoints();
            predicted_measurement_sigma_points = obj.measurementModel(sigma_points);
            predicted_measurement = predicted_measurement_sigma_points * weights_mean;
            innovation_covariance = obj.calculateCovariance(predicted_measurement_sigma_points, predicted_measurement, weights_cov, obj.R);
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

                x_new = [x(1) + x(2)*dt; x(2) + x(3)*dt; x(3)];
                %x_new = predict(x);

                sigma_points(:, i) = x_new;
            end
        end

        function predicted_measurement_sigma_points = measurementModel(obj, sigma_points)
            % Define the measurement model
            % This should be replaced with the actual measurement model
            for i = 1:size(sigma_points, 2)
                x = sigma_points(:, i);
                z = [x(1); x(3)];
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



        function x_new = getApogee(obj, x0) % Add obj as the first parameter
            % x0 is a vector containing the vertical components of
            % position, velocity, and acceleration.
            % Should be Up +ve

            x = x0(1); % Initial altitude
            xdot = x0(2); % Initial velocity
            xddot = x0(3); % Initial acceleration

            rho = obj.get_density(x); % Get current estimate for density based on altitude
            g = obj.get_gravity(x); % Get current estimate for gravity based on altitude

            Cb = (rho * xdot^2) / (2 * (xddot + g)); % Calculate the ballistic coefficient of the rocket

            while xdot > 0 && x > 0 && x < 5000
                % Propagate each particle through the prediction algorithm
                % This is a constant Cb model

                rho = obj.get_density(x);
                g = obj.get_gravity(x);

                xddot = g + ((rho * xdot^2)/(2 * Cb)); % Gravity + acceleration due to drag
                xdot = xdot + obj.dt * xddot; % Velocity = Velocity + dt * acceleration
                x = x + obj.dt * xdot; % Altitude = altitude + dt * velocity
            end

            
            x_new = [x, xdot, xddot];

        end
        %% Functions used within the propagate function
        function rho = get_density(obj, h)
            % Returns atmospheric density as a function of altitude
            % Accurate up to 11km
            % https://en.wikipedia.org/wiki/Density_of_air

            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % molar mass of dry air
            R = 8.31445; % ideal gas constant
            T_0 = 288.15; % Standard sea level temperature 288.15
            L = 0.0065; % temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R * L)) - 1); % -g used as g is -ve by default
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
