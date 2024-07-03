classdef kalmanFilterCV
    properties
            x % State vector
            P % Initial covariance
            F % State transition matrix, cv model
            Q % Process noise matrix
            H % Observation model
            R_baro % Measurement noise 
            R_Q % Process noise turning paramter
            type % CV or CA
    end
    
    methods
        function obj = kalmanFilterCV(alt_init, sigma_alt, dt)
            % Constructor to initialize the Kalman filter
            obj.x = [alt_init; 0];
            obj.R_baro = sigma_alt * sigma_alt;
            obj.R_Q = 25; %Tuning Parameter
            obj.P = eye(2);
            obj.F = [1, dt; 0, 1];
            obj.Q = obj.R_Q * [(dt^4/4), (dt^3)/2; (dt^3/2), (dt^2)];
            obj.H = [1, 0];
            obj.type = "CV";

        end
        
        function obj = predict(obj, dt)
            % Recalculate Q
            obj.Q = obj.R_Q * [(dt^4/4), (dt^3)/2;
                   (dt^3/2), (dt^2)];
            
            % Recalculate F
            obj.F = [1, dt; 0, 1];

            % Prediction step
            obj.x = obj.F * obj.x;
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
        end
        
        function obj = update(obj, z_b)
            % Barometer update step
            y_b = z_b - obj.H * obj.x;  % Innovation or measurement residual
            S_b = obj.H * obj.P * obj.H' + obj.R_baro;  % Innovation (or residual) covariance
            K_b = obj.P * obj.H' / S_b;  % Optimal Kalman gain
            obj.x = obj.x + K_b * y_b;  % Updated state estimate
            obj.P = (eye(size(obj.P)) - K_b * obj.H) * obj.P;  % Updated estimate covariance
        end
        

        function x = get_states(obj)
            x = obj.x;
        end
    end
end
