classdef EKF_constant_Cb
    % Author : Jack Johnston
    % Date : 31/07/24

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
        function obj = EKF_constant_Cb(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
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
            % EKF prediction step
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;
            obj.Q = calculateProcessNoise(obj, dt);
            % Propagate the state and covariance
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, dt);

            % Update state and covariance with process noise
            obj.x = predicted_state;
            
            obj.P = predicted_covariance + obj.Q;
        end

        function [apogee, apogee_std] = get_apogee(obj)
            % Predicts apogee and associated uncertainty using multiple
            % propagations through process model
            
            propagated_x = obj.x;
            propagated_P = obj.P;
            Q_apa = obj.calculateProcessNoise(obj.dt_apa);
            while propagated_x(2) > 0 && propagated_x(3) < obj.get_gravity(propagated_x(1))
                [propagated_x, propagated_P] = obj.processModel(propagated_x, propagated_P, obj.dt_apa);
                propagated_P = propagated_P + Q_apa;
            end

            apogee = propagated_x(1);
            apogee_std = sqrt(propagated_P(1,1));
        end

        function [new_x, new_P] = processModel(obj, x, P, dt)
            % Process model to propagate state and covariance
            
            % Get rho and g
            rho = obj.get_density(x(1));
            g = obj.get_gravity(x(1));
            B = [0,0,1,0]';
            u = 0; 


            if g - x(3) > 0 % Model for ballistic state - constant ballistic coefficient
                g_0 = -9.80665; % Standard gravity
                R_e = 6371000; % Earth radius
                p_0 = 101325; % Standard sea level atmospheric pressure
                M = 0.0289652; % Molar mass of dry air
                R = 8.31445; % Ideal gas constant
                T_0 = 288.15; % Standard sea level temperature
                L = 0.0065; % Temperature lapse rate
                
                df3_dx1 = - (2*R_e^2*g_0)/(R_e + x(1))^3 - (M*p_0*x(2)^2*((L*((M*R_e^2*g_0)/(L*R*(R_e + x(1))^2) + 1))/(T_0*(1 - (L*x(1))/T_0)^((M*R_e^2*g_0)/(L*R*(R_e + x(1))^2) + 2)) + (2*M*R_e^2*g_0*log(1 - (L*x(1))/T_0))/(L*R*(R_e + x(1))^3*(1 - (L*x(1))/T_0)^((M*R_e^2*g_0)/(L*R*(R_e + x(1))^2) + 1))))/(2*R*T_0*x(4));
 
                
                F = [1, dt, 0.5*dt^2, 0;
                     0, 1, dt, 0;
                     df3_dx1, -(rho * x(2))/(x(4)), 0, (rho * x(2)^2)/(2 * x(4)^2);
                     0, 0, 0, 1];
                u = obj.get_gravity(x(1)); %Ballisitic model absorbs gravitaty as an error if not accounted for here

            else % Model for non-ballistic state - constant acceleration
                 F = [1, dt, 0.5*dt^2, 0;
                      0, 1, dt, 0;
                      0, 0, 1, 0;
                      0, 0, 0, 1];
                 u = 0;
            end

            % Propagate state
            new_x = F * x + B * u;

            % Update covariance matrix
            new_P = F * P * F' + obj.Q;
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            % Accelerometer update step

            % Adjust measurement for gravity
            measurement = measurement + obj.get_gravity(obj.x(1));
            

            % Compute jacobian of the measurement model
            H = [0, 0, 1, 0];

            % Prediction step
            [predicted_state, predicted_covariance] = obj.processModel(obj.x, obj.P, t_current - obj.t_last_update);
            obj.t_last_update = t_current;
      

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
