classdef FP_Model_IUKF
    properties
        x;  % State
        x0; %Initial conditions, used to calculate b
        P;  % State covariance
        Q;  % Process noise covariance
        dt; % Timestep
        b;  %Matrix containing the variables for the linear model
        papogees;%Predicted apogee for each of the sigma points, final altitudes
        iteration; % Prediction iteration count
    end
    
    methods
        function obj = FP_Model_IUKF()
            obj.iteration = 0; % Initialize iteration count
            obj.b = [0,0,0,0];
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

        function obj = updateLinearF(obj)
            X = obj.x0';%Initial conditions for the sigma points
            y = obj.papogees;%Predicted apogee for each of the sigma points, final altitudes
            for i = 1:length(obj.x0(:,1))
                X(i, 4) = 1;
            end
            obj.b = (X' * X)^(-1) * X' * y;%Matrix containing the linear model coefficients
        end

        % Perform linear prediction using the 4D linear model
        function [xPred, PPred] = linearPredict(obj)
            % Extracting state variables
            x = obj.x(1); % Altitude
            xdot = obj.x(2); % Velocity
            xddot = obj.x(3); % Acceleration
            
            % Assuming linear relationship coefficients
            m1 = obj.b(1); % Coefficient for altitude
            m2 = obj.b(2); % Coefficient for velocity
            m3 = obj.b(3); % Coefficient for acceleration
            c = obj.b(4); % Intercept
            
            % Linear prediction using the 4D linear model
            xPred = m1 * x + m2 * xdot + m3 * xddot + c;
            
            % Assuming constant process noise covariance for linear model
            Q_linear = 200 * eye(1); % Adjust the covariance according to system
            
            % Covariance prediction for linear model
            PPred = obj.P + Q_linear;
        end

        % Perform UKF prediction
        function [xPred, PPred, obj] = predict(obj)
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
            obj.x0 = X;
            % Propagate sigma points through the nonlinear function f
            XProp = zeros(size(X));
            for i = 1:size(X, 2)
                XProp(:, i) = obj.f(X(:, i));
            end
            
            obj.papogees = XProp(1,:)';
            % Weights for the first sigma point
            w0 = kappa/(n + kappa); %kappa = 0, not sure if that is correct
            % Weights for all other sigma points
            wi = 1/(2*(n+kappa));

            weights = w0;
            for i = 1:n
                weights = [weights, wi, wi];
            end

            % Approximate the mean and covariance of the output distribution
            xPred = 0;
            for i = 1:(2*n+1)
                xPred = xPred + weights(i) * XProp(:,i);
            end

            PPred = 0;
            for i = 1:(2*n + 1)
               PPred = PPred + weights(i) * (XProp(:,i) - xPred)*(XProp(:,i) - xPred)';                            
            end           


        end
        % Get predicted apogee altitude and sigma
        function [predicted_apogee_altitude, predicted_apogee_sigma, obj] = getApogee(obj, x0, P0, dt)
            obj.x = x0;
            obj.P = P0;
            obj.dt = dt;
            if obj.iteration >= 10
                [xPred, PPred, obj] = obj.predict();
                obj = obj.updateLinearF();
                obj.iteration = 0;
            else
                [xPred, PPred] = obj.linearPredict();
            end
            % Increment the iteration count
            obj.iteration = obj.iteration + 1;
            predicted_apogee_altitude = xPred(1);
            predicted_apogee_sigma = sqrt(PPred(1,1));
            disp(obj.iteration)
        end

    end
end