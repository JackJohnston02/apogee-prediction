classdef APA_single_particle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dt
    end

    methods
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end

        function [predicted_apogee_altitude, predicted_apogee_sigma, obj] = getApogee(obj, x, P, dt)
            %%
            %Takes in
                % Last states
                % Timestep to run simulation 
            %Returns predicted apogee
            
            %Calcualte Cd
            %F = ma
            %F_D = 0.5 * Cd * rho * A * v^2
            %a = 0.5 * Cd*A/m * rho * v^2
            %let Cd*A/m be a new constant, Cc
            %Cc = 2*a/(rho * v^2)
            %% atmosisa is too slow for FP
            %%
            
            T_0 = 260;
            L = 0.0065; 
            
            g = -9.81 * (6371e3/(6371e3 + x(1)))^2; % Gravity as function of altitude
            T = T_0 - L * x(1);
            rho =  1.225 * (1 - (0.0065 * x(1)) / T)^(9.80665 / (287.05 * 0.0065));
            
            Cc = 2 * (x(3) - g) / (rho * (x(2) * x(2))); %2.075 should really be 2.0, adapted as correction
            
            while x(2) > 0 && x(1) < 5000
             g = -9.81 * (6371e3/(6371e3 + x(1)))^2; % Gravity as function of altitude
             T = T_0 - L * x(1);
             rho =  1.225 * (1 - (0.0065 * x(1)) / T)^(9.80665 / (287.05 * 0.0065));
            
             x(1) = x(1) + x(2) * dt + 0.5 * x(3) * dt^2;
             x(2) = x(2) + x(3) * dt;
             x(3) = g + (0.5 * Cc * rho * x(2) * x(2)); %Calculate decceleration, gravity + drag
            end
            
            predicted_apogee_altitude = x(1); %Final value
            predicted_apogee_sigma = 0;
            end
    end
end