classdef SIRPF_constant_acceleration
    % Sequential Importance Resampling Particle Filter
    % Constant acceleration model
    % Includes apogee prediction model



    properties
        x           % State estimate (weighted mean)
        P       % State covariance
        xpk_1       % Particle states at time - 1
        xpk         % Particle states at current time
        wpk_1       % Particle weights at time - 1
        wpk         % Particle weights at current time
        Q           % Process noise
        sigma_Q     % STD of general Q element noise
        sigma_Q_Cb  % STD of Cb Q element noise
        R_acc       % Accelerometer measurement noise variance
        R_baro      % Barometer measurement noise variance
        N           % Number of particles
        resampling_percentage
        resampling_strategy
        Nthresh

        t_last_update   % Time of last update
        dt_apa          % Timestep of the apogee prediction method
    end

    methods

        function obj = SIRPF_constant_acceleration(initial_state, initial_covariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
            % Set the filter parameters
            obj.N = 10000;
            obj.resampling_percentage = 1;
            obj.resampling_strategy = "multinomial"; %multinomial, systematic, random_uniform, stratified
            obj.Nthresh = obj.resampling_percentage * obj.N;

            % Noise
            obj.sigma_Q = sigma_Q;
            obj.sigma_Q_Cb = sigma_Q_Cb;
            obj.R_baro = measurement_sigma_bar^2;
            obj.R_acc = measurement_sigma_acc^2;


            obj.t_last_update = t;
            obj.dt_apa = 0.01;

            % Generate particles and weights
            obj.x =  initial_state;
            obj.P = initial_covariance;
            obj.xpk_1 = zeros(4, obj.N);
            obj.xpk = zeros(4, obj.N);
            obj.wpk_1 = repmat(1/obj.N, obj.N, 1);
            obj.wpk = repmat(1/obj.N, obj.N, 1);

            for i = 1:obj.N
                obj.xpk_1(:, i) = mvnrnd(obj.x, obj.P);
            end


        end


        function [obj, state, covariance] = predict(obj, t_current)
            dt = t_current - obj.t_last_update;
            obj.t_last_update = t_current;
            
            obj.Q = obj.calculateProcessNoise(dt);

            for i = 1:obj.N
                obj.xpk(:, i) = mvnrnd(obj.processModel(obj.xpk_1(:,i), dt), obj.Q);
            end

            % Calculate the weighted mean
            obj.x = obj.xpk * obj.wpk;

            state = obj.x;
            covariance = obj.P;

            % Normalise weights
            obj.wpk = obj.wpk./sum(obj.wpk);

            % Resample
            obj = obj.resample();

            % Update notation
            obj.xpk_1 = obj.xpk;
            obj.wpk_1 = obj.wpk;
        end

        function xpk = processModel(obj, xpk_1, dt)

            g = obj.get_gravity(xpk_1(1));
            rho = obj.get_density(xpk_1(1));

            xpk(1) = xpk_1(1) + xpk_1(2) * dt + 1/2 * xpk_1(3) * dt^2;
            xpk(2) = xpk_1(2) + xpk_1(3) * dt;
            xpk(3) = xpk_1(3);
            xpk(4) = xpk_1(4);

        end

        function Q = calculateProcessNoise(obj, dt)
            % Calculate the process noise covariance matrix Q
            Q = [1/4*dt^4*obj.sigma_Q^2, 1/2*dt^3*obj.sigma_Q^2, 1/2*dt^2*obj.sigma_Q^2, 0;
                1/2*dt^3*obj.sigma_Q^2,    dt^2*obj.sigma_Q^2,    dt*obj.sigma_Q^2, 0;
                1/2*dt^2*obj.sigma_Q^2,      dt*obj.sigma_Q^2,          obj.sigma_Q^2, 0;
                0, 0, 0, obj.sigma_Q_Cb^2];
        end

        function [obj, updated_state, updated_covariance] = updateAccelerometer(obj, measurement, t_current)
            yk = measurement - 9.81;
            obj = obj.predict(t_current);

            for i = 1:obj.N
                obj.wpk(i) = mvnpdf((yk - obj.xpk(3,i)), zeros(1, 1), obj.R_acc) + 1e-60; % TODO sort this line
            end

            % Normalise weights
            obj.wpk = obj.wpk./sum(obj.wpk);

            % Resample
            obj = obj.resample();

            % Compute state estimate
            obj.x = obj.xpk*obj.wpk;

            % Update notation
            obj.xpk_1 = obj.xpk;
            obj.wpk_1 = obj.wpk;


            updated_state = obj.x;
            updated_covariance = 0;
        end

        function [obj, updated_state, updated_covariance] = updateBarometer(obj, measurement, t_current)
            yk = measurement;
            obj = obj.predict(t_current);

            for i = 1:obj.N
                obj.wpk(i) = mvnpdf((yk - obj.xpk(1,i)), zeros(1, 1), obj.R_baro) + 1e-60; % TODO sort this line;
            end

            % Normalise weights
            obj.wpk = obj.wpk./sum(obj.wpk);

            % Resample
            obj = obj.resample();

            % Compute state estimate
            obj.x = obj.xpk*obj.wpk;

            % Update notation
            obj.xpk_1 = obj.xpk;
            obj.wpk_1 = obj.wpk;


            updated_state = obj.x;
            updated_covariance = 0;
        end

        function [apogee, apogee_std] = get_apogee(obj)
            apogee = 0;
            apogee_std = sqrt(4);
        end

        function obj = resample(obj)
            N = length(obj.wpk);
            switch obj.resampling_strategy
                case 'multinomial'
                    idx = randsample(1:N, N,'true', obj.wpk);

                case 'systematic'
                    edges = min([0 cumsum(obj.wpk)'], 1);
                    u1 = rand/N;
                    edges(end) = 1;
                    [~, idx] = histc(u1:1/N:1,edges);

                case 'random_uniform'
                    idx = find(rand <= cumsum(obj.wpk), 1);

                case 'stratified'
                    [m,n] = size(obj.wpk);
                    mn = m.*n;
                    pn = obj.wpk./sum(obj.wpk(:)).*mn;
                    s = zeros(m,n);
                    r = rand(1, mn);
                    k = 0;
                    c = 0;
                    for i =1:mn
                        c = c + pn(i);
                        if c >= 1
                            a = floor(c);
                            c = c - a;
                            idx(k+[1:a]) = i;
                            k = k + a;
                        end
                        if k< mn && c >= r(k+1)
                            c = c - 1;
                            k = k + 1;
                            idx(k) = i;
                        end
                    end

                otherwise
                    errorlg('\nResampling strategy not selected.')
            end

            % Resample particles
            obj.xpk = obj.xpk(:, idx);

            % Re-weight particles
            N = size(obj.xpk,2);
            obj.wpk = repmat(1/N, N, 1);

            % Count particles
            obj.N = N;

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