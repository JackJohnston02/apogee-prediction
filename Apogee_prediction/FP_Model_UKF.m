classdef FP_Model_UKF
    properties
        x;  % State
        P;  % State covariance
        Q;  % Process noise covariance
        dt; % Timestep
    end
    
    methods
        function obj = FP_Model_UKF(x0, P0, dt)
            obj.x = x0;
            obj.P = P0;
            obj.Q = 0;
            obj.dt = dt;

        end
        
        % State transition function
        function x = f(obj, x)
            T_0 = 260;
            L = 0.0065; 
            g = -9.81 * (6371e3/(6371e3 + x(1)))^2; % Gravity as function of altitude
            T = T_0 - L * x(1);
            rho =  1.225 * (1 - (0.0065 * x(1)) / T)^(9.80665 / (287.05 * 0.0065));
            Cc = 2.075 * (x(3) - g) / (rho * (x(2) * x(2))); %2.075 should really be 2.0, adapted as correction
            while x(2) > 0 && x(1) < 5000
                g = -9.81 * (6371e3/(6371e3 + x(1)))^2; % Gravity as function of altitude
                T = T_0 - L * x(1);
                rho =  1.225 * (1 - (0.0065 * x(1)) / T)^(9.80665 / (287.05 * 0.0065));
                
                x(1) = x(1) + x(2) * obj.dt + 0.5 * x(3) * obj.dt^2;
                x(2) = x(2) + x(3) * obj.dt;
                x(3) = g + (0.5 * Cc * rho * x(2) * x(2)); %Calculate decceleration, gravity + drag
            end
        end

        function [xPred, PPred] = predict(obj)
            % Dimension of states, in this case n = 3
            n = length(obj.x);
            % General rule for UKF, number of sigma points is 2n + 1, so 7
            % Alternative selection rules have been explored, https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6057978

            kappa = 3 - n;
            % General rule for gaussian distribution (KF from ground up)

            % Going to need the sqrt of the cov matrix obj.P
            % Should be positive and semi-definite, use Cholesky
            % decomposition, (N + kappa)P = LL'
            % P matrix is always 3x3
            sqrtP = chol(obj.P);

            % Generate sigma points
            % First sigma point, just the mean
            X = obj.x;
            % Remaining sigma points, based on the covariance matrix
            for i = 1:n
                X = [X, obj.x + sqrt((n + kappa) * sqrtP(:,i)), obj.x - sqrt((n + kappa) * sqrtP(:,i))];
            end

            % Propagate sigma points through the nonlinear function f
            XProp = zeros(size(X));
            for i = 1:size(X, 2)
                XProp(:, i) = obj.f(X(:, i));
            end
            
            % Weights for the first sigma point
            w0 = kappa/(n + kappa); %kappa = 0, not sure if that is correct
            % Weights for all other sigma points
            wi = 1/(2*(n+kappa)); 
            
            weights = w0;
            for i = 1:n
                weights = [weights, wi, wi];
            end
            
            %Approximate the mean and covariance of the output distribution
            xPred = 0;
            for i = 1:(2*n+1)
                xPred = xPred + weights(i) * XProp(:,i);
            end
            
            PPred = 0;
            for i = 1:(2*n + 1)
               PPred = PPred + weights(i) * (XProp(:,i) - xPred)*(XProp(:,i) - xPred)';                            
            end           
         

        end

        function [predicted_apogee_altitude, predicted_apogee_sigma] = getApogee(obj)
            [xPred, PPred] = predict(obj);
            predicted_apogee_altitude = xPred(1);
            predicted_apogee_sigma = sqrt(PPred(1,1));
        end
    end
end

