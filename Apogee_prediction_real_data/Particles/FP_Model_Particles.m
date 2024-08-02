function [alt_mean, alt_sigma] = FP_Model_Particles(x, P, dt)
   

    %% Generate sample of particles from the posterior mean and covariance of the rocket state
    numParticles = 1;  
    P = (P + P') / 2;%make sure P is symmetric
    
    if numParticles == 1
        particles(1,:) = x;
   
    else
        particles = mvnrnd(x, P, numParticles);%Not sure if this is the correct function?
    end

    %% Perform prediction until the velocity prediction is zero
    for i = 1:numParticles
        % Assign states to initial conditions
        x = particles(i,1);
        xdot = particles(i,2);
        xddot = particles(i,3);

        rho = get_density(x);
        g = get_gravity(x);    

        Cb = (rho * xdot^2) / (2 * (xddot - g));
        
        while xdot > 0 && x > 0 && x < 5000
            % Propagate each particle through the prediction algorithm,
            % constant Cc model
            rho = get_density(x);
            g  = get_gravity(x);

            xddot = g + ((rho * xdot^2)/(2 * Cb));
            xdot = xdot + dt * xddot;
            x = x + dt * xdot + 0.5 * xddot * dt^2; % Need to confirm is taylor expansions or just straight euler integration is correct
        end

        % Reassign states at apogee to particle
        particles(i, 1) = x;
        particles(i, 2) = xdot;
        particles(i, 3) = xddot;
    end


    %% Form distribution from the propagated particles
    % Calculate Mean (should be equal to single propagated mean
    mean_particles = mean(particles(:,1));

    % Calculate Covariance
    covariance_particle = zeros(1,1);
    for i = 1:numParticles
        covariance_particle = covariance_particle + (particles(i,1)' - mean_particles') * ((particles(i,1)' - mean_particles'))'; 
    end

    covariance_particle = covariance_particle/numParticles;%Check with Joe if this is correct?

    alt_sigma = sqrt(covariance_particle);
    alt_mean = mean_particles(1);
    
end

function rho = get_density(h)
    % Returns atmospheric density as a function of altitude
    % Accurate up to 11km
    % https://en.wikipedia.org/wiki/Density_of_air

    p_0 = 101325; % Standard sea level atmospheric pressure
    M = 0.0289652; % molar mass of dry air
    R = 8.31445; % ideal gas constant
    T_0 = 288.15; % Standard sea level temperature
    L = 0.0065; % temperature lapse rate
    g = get_gravity(h);

    rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1); % -g used as g is -ve by default
end

function g = get_gravity(h)
    % Returns gravity as a function of altitude
    % Approximates the Earth's gravity assumes a perfect sphere
    
    g_0 = -9.80665; % Standard gravity
    R_e = 6371000; % Earth radius

    g = g_0 * (R_e / (R_e + h))^2;
end