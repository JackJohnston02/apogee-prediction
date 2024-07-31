classdef kalmanFilterCA
    properties
        x % State vector [position; velocity; acceleration]
        P % Initial covariance
        F % State transition matrix
        Q % Process noise matrix
        H % Observation model
        R_baro % Measurement noise for barometer
        R_Q % Process noise tuning parameter
        type % CV or CA
    end
    
    methods
        function obj = kalmanFilterCA(alt_init, sigma_alt, dt)
            % Constructor to initialize the Kalman filter
            obj.x = [alt_init; 0; 0]; % Initialize with position, velocity, and acceleration
            obj.R_baro = sigma_alt^2; % Measurement noise covariance (barometer)
            obj.R_Q = 50; % Process noise tuning parameter (adjust as needed)
            obj.P = eye(3); % Initial covariance matrix, adjust as necessary
            obj.F = [1, dt, 0.5*dt^2; 0, 1, dt; 0, 0, 1]; % State transition matrix for constant acceleration
            obj.Q = obj.R_Q * [dt^4/4, dt^3/2, dt^2/2; dt^3/2, dt^2, dt; dt^2/2, dt, 1]; % Process noise matrix
            obj.H = [1, 0, 0]; % Observation model (measuring position)
            obj.type = "CA"; % Indicate this is a constant acceleration model
        end
        
        function obj = predict(obj, dt)
            % Prediction step
            
            % Update state transition matrix F and process noise Q
            obj.F = [1, dt, 0.5*dt^2; 0, 1, dt; 0, 0, 1];
            obj.Q = obj.R_Q * [dt^4/4, dt^3/2, dt^2/2; dt^3/2, dt^2, dt; dt^2/2, dt, 1];
            
            % Predicted state estimate
            obj.x = obj.F * obj.x;
            
            % Predicted covariance estimate
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
        end
        
        function obj = update(obj, z_b)
            % Update step (measurement update)
            
            % Compute Kalman gain
            S = obj.H * obj.P * obj.H' + obj.R_baro;
            K = (obj.P * obj.H') / S;
            
            % Update state estimate
            obj.x = obj.x + K * (z_b - obj.H * obj.x);
            
            % Update covariance estimate
            obj.P = (eye(size(obj.P)) - K * obj.H) * obj.P;
            
            % Ensure P remains symmetric
            obj.P = (obj.P + obj.P') / 2;
        end
        
        function x = get_states(obj)
            % Return current state estimate
            x = obj.x;
        end
    end
end
