classdef EKF_constant_acceleration
    % Author : Jack Johnston
    % Date : 31/07/24

    % Extended Kalman Filter state observer, used for ballistic coefficient
    % observation

    % Includes apogee prediction method


    properties
        x               % State vector
        P               % State covariance matrix
        Q               % Process noise covariance matrix
        R_acc           % Accelerometer measurement noise
        R_bar           % Barometer measurment noise
        t_last_update   % Time of last update
        dt_apa          % Timstep of the apogee prediction method
        sigma_Q         % Standard deviation process noise, general
        sigma_Q_Cb      % Standard deviation of the Cb term in the Q matrix
    end

    methods
        function obj = EKF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_noise_acc, measurement_noise_bar, t)
            obj.x = initial_state;
            obj.P = initial_covariance;
            obj.Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, 1e1]);
            obj.R_acc = measurement_noise_acc;
            obj.R_bar = measurement_noise_bar;
            obj.t_last_update = t;
            obj.dt_apa = 0.01;
            obj.sigma_Q = sigma_Q;
            obj.sigma_Q_Cb = sigma_Q_Cb;
        end

        function obj = predict(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end

        function [apogee, apogee_cov] = get_apogee(obj)
            % Predicts apogee and associated uncertainty, uses multple
            % propagate steps
        end


        function obj = updateAccelerometer(obj, measurement, t_current)
            % Accelerometer update step
        end

        function obj = updateBarometer(obj, measurement, t_current)
            % Barometric update step

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

