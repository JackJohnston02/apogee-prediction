classdef APA_Single_Particle
    properties
        dt;
        log_apogee
        log_Cb
    end

    methods
        function obj = APA_Single_Particle(dt)
            obj.dt = dt;
            obj.log_apogee = [];
            obj.log_Cb = [];
        end

        function [obj, predictedApogee] = getApogee(obj, t0, x0) % Add obj as the first parameter
            % x0 is a vector containing the vertical components of
            % position, velocity, and acceleration.
            % Should be Up +ve

            x = x0(1); % Initial altitude
            xdot = x0(2); % Initial velocity
            xddot = x0(3); % Initial acceleration

            rho = obj.get_density(x); % Get current estimate for density based on altitude
            g = obj.get_gravity(x); % Get current estimate for gravity based on altitude

            %Cb = (rho * xdot^2) / (2 * (abs(xddot) + g)); % Calculate the ballistic coefficient of the rocket
            Cb = x0(4);
            while xdot > 0 && x > 0 && x < 5000
                % Propagate each particle through the prediction algorithm
                % This is a constant Cb model

                rho = obj.get_density(x);
                g = obj.get_gravity(x);

                xddot = g - ((rho * xdot^2)/(2 * Cb)); % Gravity + acceleration due to drag
                xdot = xdot + obj.dt * xddot; % Velocity = Velocity + dt * acceleration
                x = x + obj.dt * xdot; % Altitude = altitude + dt * velocity

                
            end

            predictedApogee = x;

            obj.log_Cb = [obj.log_Cb, [t0, Cb]'];
            obj.log_apogee = [obj.log_apogee, [t0, x]'];
            
        end

        function rho = get_density(obj, h) % Add obj as the first parameter
            % Returns atmospheric density as a function of altitude
            % Accurate up to 11km
            % https://en.wikipedia.org/wiki/Density_of_air

            p_0 = 101325; % Standard sea level atmospheric pressure
            M = 0.0289652; % molar mass of dry air
            R = 8.31445; % ideal gas constant
            T_0 = 288.15; % Standard sea level temperature 288.15
            L = 0.0065; % temperature lapse rate
            g = obj.get_gravity(h);

            rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R * L)) - 1); % -g used as g is -ve by default
        end

        function g = get_gravity(obj, h) % Add obj as the first parameter
            % Returns gravity as a function of altitude
            % Approximates the Earth's gravity assumes a perfect sphere

            g_0 = -9.80665; % Standard gravity
            R_e = 6371000; % Earth radius

            g = g_0 * (R_e / (R_e + h))^2;
        end
    end
end
