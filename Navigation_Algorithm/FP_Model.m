function [alt_mean, alt_sigma] = FP_Model(x, P, t, dt)
%FORWARDPROPAGATION Summary of this function goes here
%   Detailed explanation goes here
    %Calcualte Cd
    %F = ma
    %F_D = 0.5 * Cd * rho * A * v^2
    %a = 0.5 * Cd*A/m * rho * v^2
    %let Cd*A/m be a new constant, Cc
    %Cc = 2*a/(rho * v^2)

    
    g = -9.80665;

    %calculate the air density
    
    %% Generate sample of particles from the posterior mean and covariance of the rocket state
    numParticles = 1;  
    
    P = (P + P') / 2;%make sure P is symmetric
    
    %For uniform Cc and rho
    %Cc = 2 * (x(3) - g) / (rho * (x(2)*x(2)));
    %rho = 1.225 * (1 - (0.0065 * x(1)) / 288.15)^(-g / (287.05 * 0.0065));
    if numParticles == 1
        particles(1,:) = x;
   
    else
        particles = mvnrnd(x, P, numParticles);%Not sure if this is the correct function?
    end

    T_0 = 260;
    L = 0.0065; 

    %% Perform prediction until the velocity prediction is zero
    for i = 1:numParticles
        T = T_0 - L * particles(i,1);
        rho =  1.225 * (1 - (0.0065 * particles(i,1)) / T)^(9.80665 / (287.05 * 0.0065));
        Cc = 2 * (particles(i,3) - g) / (rho * (particles(i,2)*particles(i,2)));
        while particles(i,2) > 0 && particles(i,1) > 0 && particles(i,1) < 5000
            % Propagate each particle through the prediction algorithm,
            % constant Cc model
            T = T_0 - L * particles(i,1);
            rho =  1.225 * (1 - (0.0065 * particles(i,1)) / T)^(9.80665 / (287.05 * 0.0065));
            particles(i,3) = -9.81 + (0.5 * Cc * rho * particles(i,2) * particles(i,2));
            particles(i,2) = particles(i,2) + dt * particles(i,3);
            particles(i,1) = particles(i,1) + dt * particles(i,2);
        end
    end


    %% Form distribution from the propagated particles
    % Calculate Mean (should be equal to single propagated mean
    mean_particles = mean(particles(:,1));

    % Calculate Covariance
    covariance_particle = zeros(3,3);
    for i = 1:numParticles
        covariance_particle = covariance_particle + (particles(i,:)' - mean_particles') * ((particles(i,:)' - mean_particles'))'; 
        covariance_particle = covariance_particle/numParticles;%Check with Joe if this is correct?
    end
    
    %% Calculate 3sigma bound from the covariance matrix (upper and lower limits)
    sigma_particles = sqrt([covariance_particle(1,1), covariance_particle(2,2), covariance_particle(3,3)]); %find SD, only need diagonal elemtns

    alt_sigma = sigma_particles(1);
    alt_mean = mean_particles(1);
    
end

