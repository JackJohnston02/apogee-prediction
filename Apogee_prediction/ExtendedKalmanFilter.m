classdef ExtendedKalmanFilter
    properties
        x       % State vector [altitude; velocity; acceleration; Cb]
        P       % Estimation error covariance matrix
        F       % State transition matrix
        Q       % Process noise covariance matrix
        H_b     % Observation model for barometer
        R_b     % Measurement noise covariance for barometer
        H_a     % Observation model for accelerometer
        R_a     % Measurement noise covariance for accelerometer
        B       % Control input matrix (not used in this case)
        u       % Control input (not used in this case)
    end
    
    methods
        % Constructor
        function obj = ExtendedKalmanFilter(x_init, P_init, F, Q, H_b, R_b, H_a, R_a, B, u)
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
        
        % Prediction step
        function obj = predict(obj)
            % Predict state
            obj.x = obj.F * obj.x;
            
            % Update covariance matrix
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
        end
        
        % Update step with barometer measurement
        function obj = update_barometer(obj, z_b)
            % Compute Kalman gain
            K = obj.P * obj.H_b' / (obj.H_b * obj.P * obj.H_b' + obj.R_b);
            
            % Update state estimate
            obj.x = obj.x + K * (z_b - obj.H_b * obj.x);
            
            % Update covariance matrix
            obj.P = (eye(length(obj.x)) - K * obj.H_b) * obj.P;
        end
        
        % Update step with accelerometer measurement
        function obj = update_accelerometer(obj, z_a)
            % Compute Kalman gain
            K = obj.P * obj.H_a' / (obj.H_a * obj.P * obj.H_a' + obj.R_a);
            
            % Update state estimate
            obj.x = obj.x + K * (z_a - obj.H_a * obj.x);
            
            % Update covariance matrix
            obj.P = (eye(length(obj.x)) - K * obj.H_a) * obj.P;
        end
    end
end
