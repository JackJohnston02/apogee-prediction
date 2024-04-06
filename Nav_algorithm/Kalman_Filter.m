classdef Kalman_Filter
    %KALMAN_FILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        xk_1k_1         % Posterior State Estimate at time k-1
        xkk_1           % Prior State Estimate at time k
        xkk             % Posterior State Estimate at time k
        Pk_1k_1         % Posterior Error Covariance at time k-1
        Pkk_1           % Prior Error Covariance at time k-1
        Pkk             % Posterior Error Covariance at time k-1
        Q               % Process Noise Covariance Matrix
        R               % Measurement Noise Covariance Matrix
        f               % Propagation Function (Linear or Nonlinear)
        F               % State Transition (or Jacobian) Matrix
        h               % Measurement Function (Linear or Nonlinear)
        H               % Measurement (or Jacobian) Matrix
        y               % Measurement
        I               % nXn Identity Matrix
        settings
        G
    end
    
    methods
        function obj = Kalman_Filter()
            %KALMAN_FILTER Construct an instance of this class
            %   Create a Kalman filter of type obj.type

        end
    end
end

