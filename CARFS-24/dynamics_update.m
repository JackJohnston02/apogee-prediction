function [Rocket] = dynamics_update(Rocket, t, dt)
%% Function to update the dynamics of the rocket throughout flight
% Programmer(s): Jack Johnston
% copyright © 2024
%% TODO
    % improve the gravitation function g = ... 
    % add drag force, probably implment CARFS seperate function?

%% Code
    g = -9.81 * (6371e3/(6371e3 + Rocket.x(end,1)))^2; % Gravity as function of altitude
    [T, a, P, rho] = atmosisa(Rocket.x(end,1));
    
    %Calculate Mach Number
    c = sqrt(1.4 * 287 * T);
    Ma = Rocket.x(end,2)/c; %Mach no.



%% Get body drag

    if Rocket.state == "burning"
        Cd = Rocket.dragcoef_off(Ma);
    else
        Cd = Rocket.dragcoef_on(Ma);
    end
    F_drag_body = -0.5 * Cd * Rocket.area * rho * Rocket.x(end,2)^2;

%% Get airbrake drag 

    cd_times_A = Rocket.airbrake.getCdTimesA(Ma);
    F_drag_airbrakes = -0.5 * cd_times_A * rho * Rocket.x(end,2)^2;

    
    F_thrust = Rocket.thrust(t);
    %Calculate net force acting on the rocket
    F = F_thrust + F_drag_body + F_drag_airbrakes;

    %Calculate net acceleration
    u = g + F/Rocket.mass(t);

    s = Rocket.x(end,1) + Rocket.x(end,2) * dt + 0.5 * Rocket.x(end,3)* dt^2; %dS =  vt + 1/2 at^2
    v = Rocket.x(end,2) + dt * Rocket.x(end,3);%dv = at
    a = u;
    
    Rocket.x(end+1,:) = [s,v,a];
end

