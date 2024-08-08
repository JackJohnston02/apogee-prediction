classdef EKF_constant_Cb
    % Author : Jack Johnston
    % Date : 31/07/24

    properties
        x               % State vector [altitude, velocity, acceleration, ballistic coefficient]
        P               % State covariance matrix
        Q               % Process noise covariance matrix
        R_acc           % Accelerometer measurement noise covariance matrix
        R_bar           % Barometer measurement noise covariance matrix
        t_last_update   % Time of last update
        dt_apa          % Timestep of the apogee prediction method
        sigma_Q         % Standard deviation of process noise
        sigma_Q_Cb      % Standard deviation of the Cb term in the Q matrix
    end

    methods
        function obj = EKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_noise_acc, measurement_noise_bar, t)
            obj.x = initial_state;
            obj.P = initial_covariance;
            obj.Q = obj.calculateProcessNoise(sigma_Q, sigma_Q_Cb);
            obj.R_acc = measurement_noise_acc;
            obj.R_bar = measurement_noise_bar;
            obj.t_last_update = t;
            obj.dt_apa = 0.01;
            obj.sigma_Q = sigma_Q;
            obj.sigma_Q_Cb = sigma_Q_Cb;
        end

        function [obj, predicted_state, predicted_covariance] = predict(obj, t_current)
            % EKF prediction step
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;

            % Propagate the state and covariance
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, dt);

            % Update state and covariance with process noise
            obj.x = predicted_state;

            % Loosely update the ballistic coefficient when in ballistic
            % phase
            if obj.x(3) < obj.get_gravity(obj.x(1))
                obj.x(4) = (obj.get_density(obj.x(1)) * obj.x(2)^2) / (2 * (abs(obj.x(3)) + obj.get_gravity(obj.x(1))));
            end

            obj.P = predicted_covariance + obj.Q;
        end

        function [apogee, apogee_cov] = get_apogee(obj)
            % Predicts apogee and associated uncertainty using multiple
            % propagations through process model
            
            propagated_x = obj.x;
            propagated_P = obj.P;

            while propagated_x(2) > 0 && propagated_x(3) < 0
                [propagated_x, propagated_P] = obj.processModel(propagated_x, propagated_P, obj.dt_apa);
            end

            apogee = propagated_x(1);
            apogee_cov = propagated_P(1,1);
        end

        function [new_x, new_P] = processModel(obj, x, P, dt)
            % Process model to propagate state and covariance

            % Define the state transition matrix (Jacobian)
            F = [1, dt, 0.5*dt^2, 0;
                 0, 1, dt, 0;
                 0, 0, 1, 0;
                 0, 0, 0, 1];

            % Propagate state
            new_x = F * x;

            % Update covariance matrix
            new_P = F * P * F';
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            % Accelerometer update step

            % Adjust measurement for gravity
            measurement = measurement + obj.get_gravity(obj.x(1));
            obj.t_last_update = t_current;

            % Compute jacobian of the measurement model
            H = [0, 0, 1, 0];

            % Prediction step
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, t_current - obj.t_last_update);

            % Calculate kalman gain
            S = H * predicted_covariance * H' + obj.R_acc;
            K = predicted_covariance * H' / S;

            % Update state and covariance with new measurement
            innovation = measurement - H * predicted_state;
            updated_state = predicted_state + K * innovation;
            updated_covariance = predicted_covariance - K * S * K';
            obj.x = updated_state;
            obj.P = updated_covariance;
        end

        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)
            % Barometer update step

            obj.t_last_update = t_current;

            % Measurement model jacobian
            H = [1, 0, 0, 0];

            % Prediction step
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, t_current - obj.t_last_update);

            % Calculate kalman gain
            S = H * predicted_covariance * H' + obj.R_bar;
            K = predicted_covariance * H' / S;

            % Update state and covariance with new measurement
            innovation = measurement - H * predicted_state;
            updated_state = predicted_state + K * innovation;
            updated_covariance = predicted_covariance - K * S * K';
            obj.x = updated_state;
            obj.P = updated_covariance;
        end

        function Q = calculateProcessNoise(obj, sigma_Q, sigma_Q_Cb)
            % Calculate the process noise covariance matrix Q
            Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, sigma_Q_Cb]);
        end

        function rho = get_density(obj, h)
            % Returns atmospheric density as a function of altitude

            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % Molar mass of dry air
            R = 8.31445; % Ideal gas constant
            T_0 = 288.15; % Standard sea level temperature
            L = 0.0065; % Temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M) / (R * T_0) * (1 - (L * h) / (T_0))^(((-g * M) / (R * L)) - 1);
        end

        function g = get_gravity(obj, h)
            % Returns gravity as a function of altitude

            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h))^2;
        end
    end
end