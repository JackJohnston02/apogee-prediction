classdef EKF_constant_acceleration
    % Author : Jack Johnston
    % Date : 31/07/24

    % Extended Kalman Filter state observer for constant acceleration
    % Includes apogee prediction method
    
    properties
        x               % State vector [altitude, velocity, acceleration, ballistic coefficient]
        P               % State covariance matrix
        Q               % Process noise covariance matrix
        R_acc           % Accelerometer measurement noise variance
        R_bar           % Barometer measurement noise variance
        t_last_update   % Time of last update
        dt_apa          % Timestep of the apogee prediction method
        sigma_Q         % Standard deviation of process noise
        sigma_Q_Cb      % Standard deviation of the Cb term in the Q matrix
    end

    methods
        function obj = EKF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
            obj.x = initial_state; %4x1
            obj.P = initial_covariance; %4x4
            obj.Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, 1e1]); %4x4
            obj.R_acc = measurement_sigma_acc^2; %Scalar
            obj.R_bar = measurement_sigma_bar^2; %Scalar
            obj.t_last_update = t; %Scalar
            obj.dt_apa = 0.01; %Scalar
            obj.sigma_Q = sigma_Q; %Scalar
            obj.sigma_Q_Cb = sigma_Q_Cb; %Scalar
        end

        function [obj, predicted_state, predicted_covariance] = predict(obj, t_current)
            % EKF prediction step
            dt = t_current - obj.t_last_update; % Scalar
            obj.t_last_update = t_current; %Scalar
            
            obj.Q = calculateProcessNoise(obj, dt); %4x4
            % Propagate the state and covariance
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, dt);

            % Update state and covariance with process noise
            obj.x = predicted_state; % 4x1 

            obj.P = predicted_covariance + obj.Q;% 4X4
        end

        function [apogee, apogee_std] = get_apogee(obj)
            % Predicts apogee and associated uncertainty using multiple
            % propagations through process model
            
            propagated_x = obj.x; % 4x1
            propagated_P = obj.P; % 4x4
            Q_apa = obj.calculateProcessNoise(obj.dt_apa); % 4x4
            while propagated_x(2) > 0 && propagated_x(3) < 0
                [propagated_x, propagated_P] = obj.processModel(propagated_x, propagated_P, obj.dt_apa); %[(4x1), (4x4)]
                 propagated_P = propagated_P + Q_apa; % 4x4
            end

            apogee = propagated_x(1); %Select altitude state form propagated state
            apogee_std = sqrt(propagated_P(1,1)); % Select the element in the covariance matrix corresponding to altitude (1,1)
        end

        function [new_x, new_P] = processModel(obj, x, P, dt)
            % Process model to propagate state and covariance
            

            % Get rho and g
            rho = obj.get_density(x(1)); %Scalar
            g = obj.get_gravity(x(1)); %Scalar


            % Recalculate Cb everytime as not observered in the constant
            % acceleration model
            B = [0, 0, 0, 1]'; %4x1 vector
            u = (rho * x(2)^2) / (2 * -(x(3) - g)); %This is a scalar calculation of Cb, completely incorrect to do it this way!


            % Define the state transition matrix (Jacobian)
            F = [1, dt, 0.5*dt^2, 0;
                 0, 1, dt, 0;
                 0, 0, 1, 0;
                 0, 0, 0, 0]; %4x4 Jacobian

            % Propagate state
            new_x = F * x + B * u; % new 4x1 vector

            % Update covariance matrix
            new_P = F * P * F'; % New 4x4 matrix
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            % Accelerometer update step

            % Adjust measurement for gravity
            measurement = measurement + obj.get_gravity(obj.x(1));%Scalar = Scalar + Scalar
            obj.t_last_update = t_current; %Scalar

            % Compute jacobian of the measurement model
            H = [0, 0, 1, 0]; % Row vecotor selects sate for measurement

            % Prediction step
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, t_current - obj.t_last_update);%Pass state and covariance through the process model

            % Calculate kalman gain
            S = H * predicted_covariance * H' + obj.R_acc; %scalar = 1x4 * 4x4 * 1x4' + Scalar
            K = predicted_covariance * H' / S; % 4x1 = 4x4 * 1x4' / Scalar

            % Update state and covariance with new measurement
            innovation = measurement - H * predicted_state; %Scalar = Scalar - 1x4 * 4x1
            updated_state = predicted_state + K * innovation; %4x1 = 4x1 + 4x1 * Scalar
            updated_covariance = predicted_covariance - K * S * K'; %4x4 = 4x4 - 4x1 * Scalar * 4x1'
            obj.x = updated_state; %4x1
            obj.P = updated_covariance; %4x4
        end

        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)
            % Reference the updateAccelerometer for the matrix dimensions,
            % different matrixes, same dimensions.

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

         function Q = calculateProcessNoise(obj, dt)
            % Calculate the process noise covariance matrix Q
            Q = [1/4*dt^4*obj.sigma_Q^2, 1/2*dt^3*obj.sigma_Q^2, 1/2*dt^2*obj.sigma_Q^2, 0;
                 1/2*dt^3*obj.sigma_Q^2,    dt^2*obj.sigma_Q^2,    dt*obj.sigma_Q^2, 0;
                 1/2*dt^2*obj.sigma_Q^2,      dt*obj.sigma_Q^2,          obj.sigma_Q^2, 0;
                 0, 0, 0, obj.sigma_Q_Cb^2];
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
