classdef APA_IUKF
    % FP_Model_IUKF - Class for performing Unscented Kalman Filter (UKF) predictions
    % Accelation, Velocity, Altitude use to calculate linear model coefficient
    % This class implements a model for performing predictions using the Unscented Kalman Filter (UKF).
    % It includes methods for state transition, linear prediction using a linear model, and performing
    % UKF prediction. The class also provides functionality for updating the linear model coefficients based on the sigma particles' initial conditions for the acceleration, velocity, and position.
    % The object switches between the UKF and the linear prediction based on a
    % predefined frequency.
    %
    % Properties:
    %   x             - State vector representing altitude, velocity, and acceleration
    %   x0            - Initial conditions used to calculate linear model coefficients
    %   P             - State covariance matrix
    %   Q             - Process noise covariance matrix
    %   dt            - Timestep
    %   b             - Matrix containing the variables for the linear model
    %   papogees      - Predicted apogee for each of the sigma points (final altitudes)
    %   iteration     - Prediction iteration count
    %
    % Methods:
    %   FP_Model_IUKF - Constructor method to initialize the class object
    %   f             - State transition function
    %   updateLinearF - Update linear model coefficients
    %   linearPredict - Perform linear prediction using the linear model
    %   predict       - Perform UKF prediction
    %   getApogee     - Get predicted apogee altitude and sigma
    %
    % Author: Jack Johnston
    % Date: 26/05/24

    properties
        x;  % State
        x0; % Initial conditions, used to calculate linear model coefficients
        P;  % State covariance
        P0; % Initial covariance, from the last UKF update
        Q;  % Process noise covariance
        dt; % Timestep
        b;  % Matrix containing the variables for the linear model
        papogees; % Predicted apogee for each of the sigma points, final altitudes
        iteration; % Prediction iteration count
        iteration_period; % Length of one UKF/ Linear function cycle
    end

    methods
        function obj = APA_IUKF()
            obj.iteration_period = 50;
            obj.iteration = obj.iteration_period; % Initialize iteration count, ensure first prediction is performed by the UKF
            obj.b = zeros(4, 1); % Initialize coefficients for the linear model
        end
        
        % State transition function
        function x = f(obj, x)
            T_0 = 260;
            L = 0.0065; 
            g = -9.81 * (6371e3 / (6371e3 + x(1)))^2; % Gravity as function of altitude
            T = T_0 - L * x(1);
            rho = 1.225 * (1 - (0.0065 * x(1)) / T)^(9.80665 / (287.05 * 0.0065));
            Cc = 2.075 * (x(3) - g) / (rho * (x(2) * x(2))); % 2.075 should really be 2.0, adapted as correction
            while x(2) > 0 && x(1) < 5000
                g = -9.81 * (6371e3 / (6371e3 + x(1)))^2; % Gravity as function of altitude
                T = T_0 - L * x(1);
                rho = 1.225 * (1 - (0.0065 * x(1)) / T)^(9.80665 / (287.05 * 0.0065));

                x(1) = x(1) + x(2) * obj.dt + 0.5 * x(3) * obj.dt^2;
                x(2) = x(2) + x(3) * obj.dt;
                x(3) = g + (0.5 * Cc * rho * x(2) * x(2)); % Calculate deceleration, gravity + drag
            end
        end

        function obj = updateLinearF(obj)
            % Transpose initial conditions for the sigma points
            X = obj.x0';
            
            % Predicted apogee for each of the sigma points (final altitudes)
            y = obj.papogees;
            
            % Add a column of ones for the intercept term
            X = [ones(size(X, 1), 1), X];
            
            % Calculate the coefficients for the linear model using the normal equation
            obj.b = (X' * X) \ (X' * y); % Using \ instead of inv(X' * X) for better numerical stability
        end

        % Perform linear prediction using the linear model
        function [xPred, PPred] = linearPredict(obj)
            % Extracting state variables
            x = obj.x(1);   % Altitude
            xdot = obj.x(2); % Velocity
            xddot = obj.x(3); % Acceleration
            
            % Construct the feature vector for the current state
            x_features = [1, x, xdot, xddot]; % Include the intercept term
            
            % Calculate the prediction using the fitted model coefficients
            xPred = x_features * obj.b;
            
            % Assuming constant process noise covariance for linear model
            Q_linear = obj.iteration * 10 * eye(1); % Adjust the covariance according to the system
            
            % Covariance prediction for linear model
            obj.P0 = obj.P0 + Q_linear;
            PPred = obj.P0;
        end

        % Perform UKF prediction
        function [xPred, PPred, obj] = predict(obj)
            % Dimension of states, in this case n = 3
            n = length(obj.x);
            % General rule for UKF, number of sigma points is 2n + 1, so 7
            kappa = 3 - n;

            % Going to need the sqrt of the cov matrix obj.P
            sqrtP = chol(obj.P);

            % Generate sigma points
            % First sigma point, just the mean
            obj.x = obj.x';
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
            
            % Weights for the sigma points
            w0 = kappa / (n + kappa);
            wi = 1 / (2 * (n + kappa));
            weights = [w0, repelem(wi, 2 * n)];

            % Approximate the mean and covariance of the output distribution
            xPred = XProp * weights';
            
            PPred = zeros(n, n);
            for i = 1:(2 * n + 1)
                diff = XProp(:, i) - xPred;
                PPred = PPred + weights(i) * (diff * diff');
            end
        end

        % Get predicted apogee altitude and sigma
        function [predicted_apogee_altitude, predicted_apogee_sigma, obj] = getApogee(obj, x, P, dt)
            obj.x = x;
            obj.P = P;
            obj.dt = dt;
            if obj.iteration >= obj.iteration_period
                [xPred, PPred, obj] = obj.predict();
                obj.P0 = PPred;
                obj = obj.updateLinearF();
                obj.iteration = 0;
            else
                [xPred, PPred] = obj.linearPredict();
            end
            % Increment the iteration count
            obj.iteration = obj.iteration + 1;
            predicted_apogee_altitude = xPred(1);
            predicted_apogee_sigma = sqrt(PPred(1,1));
        end

    end
end
