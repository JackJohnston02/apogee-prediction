function [F] = u(x,t)


burn_time = 3;
average_thrust = 200;


forces = zeros(2, 100);

for i = 1:burn_time
        forces(1, i) = average_thrust;
end


F = forces(x,t);
end

