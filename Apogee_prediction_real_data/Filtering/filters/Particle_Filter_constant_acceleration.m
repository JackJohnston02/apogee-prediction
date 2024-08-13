classdef Particle_Filter_constant_acceleration
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
%% Constructor
        function obj = Particle_Filter_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_noise_acc, measurement_noise_bar, t)
        obj.x = initial_state;              
        obj.P = initial_covariance;              
        obj.Q = sigma_Q * diag([1e-3, 1e-3, 1e-2, 1e1]);             
        obj.R_acc = measurement_sigma_acc^2;      
        obj.R_bar = measurement_sigma_bar^2;         
        obj.t_last_update = t;
        obj.dt_apa = 0.01;   
        obj.sigma_Q = sigma_Q; % Initialize obj.sigma_Q       
        obj.sigma_Q_Cb = sigma_Q_Cb;% Scaler for Cb Q matrix      
        end
%% Methods used by main script
         function [obj, predicted_state, predicted_covariance] = predict(obj, t_current)
            % Prediction step
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;
         end

        
         function [apogee, apogee_cov] = get_apogee(obj)
            % Predicts apogee and associated uncertainty using multiple
            % propagations through process model
        end
        
        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
          
        end


        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)

        end

%% Internal Methods
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
