function [Rocket] = dynamics_update(Rocket, t, dt)
    Rocket.x(end+1,1) = Rocket.x(end,1) + Rocket.x(end,2) * dt + 0.5 * Rocket.x(end,3)^2; %dS =  vt + 1/2 at^2
    Rocket.x(end+1,2) = Rocket.x(end,2) + dt * Rocket.x(end,3);%dv = at
    Rocket.x(end+1,3) = Rocket.x(end,3);

end

