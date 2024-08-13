classdef SIRPF_constant_acceleration
    % Sequential Importance Resampling Particle Filter for constant acceleration model
    % This class mirrors the interface of the UKF_constant_acceleration class.

    properties
        xpk_1           % Particles from previous time step
        xpk             % Particles for current time step
        wpk_1           % Weights from previous time step
        wpk             % Weights for current time step
        Q               % Process noise covariance matrix
        R_acc           % Accelerometer measurement noise variance
        R_bar           % Barometer measurement noise variance
        N               % Number of particles
        n               % State dimension
        m               % Measurement dimension
        xhat            % Estimated state
        t_last_update   % Time of last update
        dt_apa          % Timestep of the apogee prediction method
        resampling_percentage % Resampling threshold
        resampling_strategy   % Resampling strategy
        Nthresh         % Resampling threshold count
    end

    methods
        function obj = SIRPF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
            obj.n = length(initial_state);
            obj.m = 1; % Assuming 1D measurements (e.g., acceleration, altitude)
            obj.N = 1000; % Number of particles

            % Initialize particles
            obj.xpk_1 = mvnrnd(initial_state, initial_covariance, obj.N)'; % Particles initialized from initial state
            obj.xpk = zeros(size(obj.xpk_1));
            
            % Initialize weights
            obj.wpk_1 = repmat(1/obj.N, obj.N, 1);
            obj.wpk = obj.wpk_1;

            % Initialize process and measurement noise
            obj.Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, sigma_Q_Cb^2]);
            obj.R_acc = measurement_sigma_acc^2;
            obj.R_bar = measurement_sigma_bar^2;
            obj.t_last_update = t;

            % Resampling settings
            obj.resampling_percentage = 0.5;
            obj.resampling_strategy = 'multinomial';
            obj.Nthresh = obj.resampling_percentage * obj.N;

            obj.dt_apa = 0.01; % Timestep for apogee prediction
        end

        function [obj, predicted_state, predicted_covariance] = predict(obj, t_current)
            % Prediction step of the SIR particle filter

            % Calculate time step
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;

            % Propagate particles
            for i = 1:obj.N
                obj.xpk(:, i) = mvnrnd(obj.processModel(obj.xpk_1(:, i), dt), obj.Q);
            end

            % Recompute state estimate
            obj.xhat = obj.xpk * obj.wpk;

            % Output current state estimate and covariance
            predicted_state = obj.xhat;
            predicted_covariance = obj.calculateCovariance(obj.xpk, obj.xhat, obj.wpk);

            % Update previous particles and weights
            obj.xpk_1 = obj.xpk;
            obj.wpk_1 = obj.wpk;
        end

        function [apogee, apogee_std] = get_apogee(obj)
            apogee = 0;
            apogee_std = sqrt(4);
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            % Accelerometer measurement update step

            % Pre-processing (compensating for gravity)
            measurement = measurement + obj.get_gravity(obj.xhat(1));

            % Update weights based on measurement likelihood
            for i = 1:obj.N
                measurement_predict = obj.measurementModelAccelerometer(obj.xpk(:, i));
                obj.wpk(i) = obj.wpk_1(i) * mvnpdf(measurement, measurement_predict, obj.R_acc);
            end

            % Normalize weights
            obj.wpk = obj.wpk / sum(obj.wpk);

            % Resample if necessary
            if 1 / sum(obj.wpk.^2) < obj.Nthresh
                obj = obj.resample();
            end

            % Recompute state estimate and covariance
            obj.xhat = obj.xpk * obj.wpk;
            updated_state = obj.xhat;
            updated_covariance = obj.calculateCovariance(obj.xpk, obj.xhat, obj.wpk);

            % Loosely calculate the ballistic coefficient
            if obj.xhat(3) < obj.get_gravity(obj.xhat(1))
                obj.xhat(4) = (obj.get_density(obj.xhat(1)) * obj.xhat(2)^2) / (2 * (abs(obj.xhat(3)) + obj.get_gravity(obj.xhat(1))));
            end
        end

        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)
            % Barometer measurement update step

            % Update weights based on measurement likelihood
            for i = 1:obj.N
                measurement_predict = obj.measurementModelBarometer(obj.xpk(:, i));
                obj.wpk(i) = obj.wpk_1(i) * mvnpdf(measurement, measurement_predict, obj.R_bar);
            end

            % Normalize weights
            obj.wpk = obj.wpk / sum(obj.wpk);

            % Resample if necessary
            if 1 / sum(obj.wpk.^2) < obj.Nthresh
                obj = obj.resample();
            end

            % Recompute state estimate and covariance
            obj.xhat = obj.xpk * obj.wpk;
            updated_state = obj.xhat;
            updated_covariance = obj.calculateCovariance(obj.xpk, obj.xhat, obj.wpk);
        end

        function predicted_measurement_sigma_points = measurementModelAccelerometer(obj, x)
            % Measurement model for the accelerometer
            predicted_measurement_sigma_points = x(3);
        end

        function predicted_measurement_sigma_points = measurementModelBarometer(obj, x)
            % Measurement model for the barometer
            predicted_measurement_sigma_points = x(1);
        end

        function sigma_points = processModel(obj, x, dt)
            % Process model for state propagation
            g = obj.get_gravity(x(1));
            rho = obj.get_density(x(1));

            % Update state based on constant acceleration model
            x(1) = x(1) + x(2) * dt + 1/2 * x(3) * dt^2;
            x(2) = x(2) + x(3) * dt;
            x(3) = x(3);
            x(4) = x(4); % Ballistic coefficient remains unchanged

            sigma_points = x;
        end

        function covariance = calculateCovariance(obj, x, mean, weights)
            % Calculate the weighted covariance of the particles
            n = size(x, 1);
            covariance = zeros(n, n);
            for i = 1:obj.N
                diff = x(:, i) - mean;
                covariance = covariance + weights(i) * (diff * diff');
            end
        end

        function obj = resample(obj)
            % Resample particles according to the weights

            switch obj.resampling_strategy
                case 'multinomial'
                    idx = randsample(1:obj.N, obj.N, true, obj.wpk);
                otherwise
                    error('Unsupported resampling strategy');
            end

            % Resample particles
            obj.xpk = obj.xpk(:, idx);

            % Reset weights
            obj.wpk = repmat(1/obj.N, obj.N, 1);
        end

        function rho = get_density(obj, h)
            % Returns atmospheric density as a function of altitude
            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % molar mass of dry air
            R = 8.31445; % ideal gas constant
            T_0 = 288.15; % Standard sea level temperature
            L = 0.0065; % temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1);
        end

        function g = get_gravity(obj, h)
            % Returns gravity as a function of altitude
            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h))^2;
        end
    end
end
