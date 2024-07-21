classdef Observer_EKF
    properties
        x          % State vector
        P              % State covariance matrix
        t_last_prediction % Time of last prediction step
        Q              % Process noise covariance matrix
        R_acc          % Accelerometer measurement noise covariance matrix
        R_bar          % Barometer measurement noise covariance matrix
    end

    methods
        function obj = Observer_EKF(initial_state, initial_covariance, process_noise, measurement_noise_acc, measurement_noise_bar, current_time)
            obj.x = initial_state;
            obj.P = initial_covariance;
            obj.Q = process_noise;
            obj.R_acc = measurement_noise_acc;
            obj.R_bar = measurement_noise_bar;
            obj.t_last_prediction = current_time;
        end

        %% Prediction step
        function [obj, predicted_state, predicted_covariance] = predict(obj, t)
            dt = t - obj.t_last_prediction;
            obj.t_last_prediction = t;

            rho = obj.get_density(obj.x(1));

            % Construct Jacobian
            F = [1, dt, 0;
                0, (1 - dt * ((rho * obj.x(2))/obj.x(3))), (dt * ((rho * obj.x(3)^2)/(2*obj.x(3)^2)));
                0, 0, 1];

            % Construct process noise matrix
            obj.Q = 1e1*[(dt^5)/20, (dt^4)/8, (dt^3)/6;
                     (dt^4)/8, (dt^3)/3, (dt^2)/2;
                    (dt^3)/6, (dt^2)/2, dt];
                
            % Update state
            obj.x = F * obj.x; % Predicted state
            predicted_state = obj.x;
            
            % Update covariance
            obj.P = F * obj.P * F' + obj.Q; % Predicted covariance
            predicted_covariance = obj.P;
        end


        %% Accelerometer update  step
        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement)
            obj.x(3) = 1;
            
            updated_state = obj.x;
            updated_covariance = obj.P;
        end


        %% Barometer update  step
        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement)
           obj.x(1) = measurement;
            
            updated_state = obj.x;
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

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1); % -g used as g is -ve by default
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
