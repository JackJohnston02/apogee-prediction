classdef Observer_EKF
    properties
        x          % State vector
        P          % State covariance matrix
        t_last_prediction % Time of last prediction step
        Q_s          % Process noise covariance scaling parameter
        R_acc      % Accelerometer measurement noise covariance matrix
        R_bar      % Barometer measurement noise covariance matrix
    end

    methods
        function obj = Observer_EKF(initial_state, initial_covariance, process_noise, measurement_noise_acc, measurement_noise_bar, current_time)
            obj.x = initial_state;
            obj.P = initial_covariance;
            obj.Q_s = process_noise;
            obj.R_acc = measurement_noise_acc;
            obj.R_bar = measurement_noise_bar;
            obj.t_last_prediction = current_time;
        end

        %% Prediction step
        function [obj, predicted_state, predicted_covariance] = predict(obj, t)
            dt = t - obj.t_last_prediction;
            obj.t_last_prediction = t;

            rho = 1.293;
            g = -9.81;

            % Construct Jacobian
            F = [1, dt, 0;
                0, 1 - (dt *( rho * obj.x(2))/obj.x(3)), dt * ( rho * obj.x(2)^2/(2 * obj.x(3)^2));
                0, 0, 1];

            % Construct process noise matrix
            Q = obj.Q_s * [(dt^5)/20, (dt^4)/8, 0;
                            (dt^4)/8, (dt^3)/3, 0;
                            0, 0, dt^2];
               

            % Update state
            obj.x = F * obj.x; % Predicted state
            predicted_state = obj.x;
            
            % Update covariance
            obj.P = F * obj.P * F' + Q; % Predicted covariance
            predicted_covariance = obj.P;
        end

        %% Accelerometer update step
        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement)
            rho = obj.get_density(obj.x(1));

            % Need to conver acceleration measurement to ballistic coefficient
            measurement = (rho * obj.x(2)^2)/(2*(measurement));
            H = [0, 0, 1]; % Measurement matrix
            y = measurement - H * obj.x; % Measurement residual
            S = H * obj.P * H' + obj.R_acc; % Residual covariance
            K = obj.P * H' / S; % Kalman gain
            
            % Update state
            obj.x = obj.x + y *K;
            updated_state = obj.x;
            
            % Update covariance
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
            updated_covariance = obj.P;
        end

        %% Barometer update step
        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement)
            H = [1, 0, 0]; % Measurement matrix
            y = measurement - H * obj.x; % Measurement residual
            S = H * obj.P * H' + obj.R_bar; % Residual covariance
            K = obj.P * H' / S; % Kalman gain

            % Update state
            obj.x = obj.x + K * y;
            updated_state = obj.x;
            
            % Update covariance
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
            updated_covariance = obj.P;
        end

        %% Density method
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

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R * L)) - 1); % -g used as g is -ve by default
        end

        %% Gravity method
        function g = get_gravity(obj, h)
            % Returns gravity as a function of altitude
            % Approximates the Earth's gravity assumes a perfect sphere

            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h))^2;
        end
    end
end
