function [Rocket] = dynamics_update(Rocket, t, dt)
%% Function to update the dynamics of the rocket throughout flight
% Programmer(s): Jack Johnston
% copyright © 2024
%% TODO
    % improve the gravitation function g = ... 
    % add drag force, probably implment CARFS seperate function?

%% Code
    g = -9.81 * (6371e3/(6371e3 + Rocket.x(end,1)))^2;
    
    F_thrust = Rocket.thrust(t);
    F_drag_body = -0.001 * Rocket.x(end,2)^2;
    F_drag_airbrakes = 0;

    %Calculate total force acting on the rocket
    F = F_thrust + F_drag_body + F_drag_airbrakes;

    u = g + F/Rocket.mass(t);

    s = Rocket.x(end,1) + Rocket.x(end,2) * dt + 0.5 * Rocket.x(end,3)* dt^2; %dS =  vt + 1/2 at^2
    v = Rocket.x(end,2) + dt * Rocket.x(end,3);%dv = at
    a = u;
    
    Rocket.x(end+1,:) = [s,v,a];
end

