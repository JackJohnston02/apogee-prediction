classdef FP_Model_UKF
    properties
        x;  % State
        P;  % State covariance
        Q;  % Process noise covariance
        dt; % Timestep
    end
    
    methods
        function obj = FP_Model_UKF(dt)
            obj.dt = dt;
        end

        % State transition function
        function x_new = f(obj, x0)
            x = x0(1);
            xdot = x0(2);
            xddot = x0(3);
            dt = obj.dt;

            rho = obj.get_density(x);
            g = obj.get_gravity(x);

            Cb = (rho * xdot^2) / (2 * (xddot - g));

            while xdot > 0 && x > 0 && x < 5000
                % Propagate each particle through the prediction algorithm,
                % constant Cc model
                rho = obj.get_density(x);
                g  = obj.get_gravity(x);

                xddot = g + ((rho * xdot^2)/(2 * Cb));
                xdot = xdot + dt * xddot;
                x = x + dt * xdot;
            end

            % Reassign states at apogee to particle
            x_new = [x; xdot; xddot];
        end

        function [xPred, PPred] = predict(obj)
            % Dimension of states, in this case n = 3
            n = length(obj.x);
            % General rule for UKF, number of sigma points is 2n + 1, so 7

            % Calculate coefficients for the UKF
            alpha = 1;  % from The Unscented Kalman Filter for Nonlinear Estimation
            beta = 2;      % from The Unscented Kalman Filter for Nonlinear Estimation
            kappa = 0;     % from The Unscented Kalman Filter for Nonlinear Estimation
            lambda = alpha^2 * (n + kappa) - n;  % from Bayesian Filtering and Smoothing - Simo Sarkka

            % Going to need the sqrt of the cov matrix obj.P
            % Should be positive and semi-definite, use Cholesky decomposition, (N + kappa)P = LL'
            % P matrix is always 3x3
            sqrtP = chol(obj.P);

            % Generate sigma points
            % First sigma point, just the mean
            X = obj.x;
            % Remaining sigma points, based on the covariance matrix
            for i = 1:n
                X = [X, obj.x + sqrt((n + lambda)) * sqrtP(:, i), obj.x - sqrt((n + lambda)) * sqrtP(:, i)];
            end

            % Propagate sigma points through the nonlinear function f
            XProp = zeros(size(X));
            for i = 1:size(X, 2)
                XProp(:, i) = obj.f(X(:, i));
            end
            
            % Weights for mean estimate for the first sigma point from Bayesian Filtering and Smoothing - Simo Sarkka
            w0 = lambda / (n + lambda);
            % Weights for mean estimate for all other sigma points
            wi = 1 / (2 * (n + lambda)); 
            weights = w0;
            for i = 1:n
                weights = [weights, wi, wi];
            end
    
            % Calculate the mean of the output distribution
            xPred = 0;
            for i = 1:(2 * n + 1)
                xPred = xPred + weights(i) * XProp(:, i);
            end
            
            % Weights for covariance estimate for the first sigma point from Bayesian Filtering and Smoothing - Simo Sarkka
            w0 = lambda / (n + lambda) + (1 - alpha^2 + beta);
            % Weights for mean estimate for all other sigma points
            wi = 1 / (2 * (n + lambda)); 
            weights = w0;
            for i = 1:n
                weights = [weights, wi, wi];
            end

            % Calculate the covariance of the output distribution
            PPred = 0;
            for i = 1:(2 * n + 1)
                PPred = PPred + weights(i) * (XProp(:, i) - xPred) * (XProp(:, i) - xPred)';                            
            end
        end

        function [predicted_apogee_altitude, predicted_apogee_sigma] = getApogee(obj, x0, P0)
            obj.x = x0;
            obj.P = P0;

            [xPred, PPred] = obj.predict();
            predicted_apogee_altitude = xPred(1);
            predicted_apogee_sigma = sqrt(PPred(1, 1));
        end

        function rho = get_density(obj, h)
            % Returns atmospheric density as a function of altitude
            % Accurate up to 11km
            % https://en.wikipedia.org/wiki/Density_of_air

            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % molar mass of dry air
            R = 8.31445; % ideal gas constant
            T_0 = 288.15; % Standard sea level temperature
            L = 0.0065; % temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1); % -g used as g is -ve by default
        end

        function g = get_gravity(obj, h)
            % Returns gravity as a function of altitude
            % Approximates the Earth's gravity assumes a perfect sphere

            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h))^2;
        end
    end
end
