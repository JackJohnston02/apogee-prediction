classdef KalmanFilter
    properties
        x   % State vector
        P   % State covariance matrix
        F   % State transition matrix
        Q   % Process noise covariance matrix
        H_b % Observation model for barometer
        R_b % Measurement noise covariance matrix for barometer
        H_a % Observation model for accelerometer
        R_a % Measurement noise covariance matrix for accelerometer
        B   % Control input matrix
        u   % Control input
    end
    
    methods
        function obj = KalmanFilter(x_init, P_init, F, Q, H_b, R_b, H_a, R_a, B, u)
            % Constructor to initialize the Kalman filter
            obj.x = x_init;
            obj.P = P_init;
            obj.F = F;
            obj.Q = Q;
            obj.H_b = H_b;
            obj.R_b = R_b;
            obj.H_a = H_a;
            obj.R_a = R_a;
            obj.B = B;
            obj.u = u;
        end
        
        function obj = predict(obj)
            % Prediction step
            obj.x = obj.F * obj.x + obj.B * obj.u;
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
        end
        
        function obj = update_barometer(obj, z_b)
            % Barometer update step
            y_b = z_b - obj.H_b * obj.x;  % Innovation or measurement residual
            S_b = obj.H_b * obj.P * obj.H_b' + obj.R_b;  % Innovation (or residual) covariance
            K_b = obj.P * obj.H_b' / S_b;  % Optimal Kalman gain
            obj.x = obj.x + K_b * y_b;  % Updated state estimate
            obj.P = (eye(size(obj.P)) - K_b * obj.H_b) * obj.P;  % Updated estimate covariance
        end
        
        function obj = update_accelerometer(obj, z_a)
            % Accelerometer update step
            y_a = z_a - obj.H_a * obj.x;  % Innovation or measurement residual
            S_a = obj.H_a * obj.P * obj.H_a' + obj.R_a;  % Innovation (or residual) covariance
            K_a = obj.P * obj.H_a' / S_a;  % Optimal Kalman gain
            obj.x = obj.x + K_a * y_a;  % Updated state estimate
            obj.P = (eye(size(obj.P)) - K_a * obj.H_a) * obj.P;  % Updated estimate covariance
        end
    end
end
