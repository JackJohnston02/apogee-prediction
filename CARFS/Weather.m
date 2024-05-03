function [Wind_F, time_exposed] = Weather(rho,Wind_S,delta_t,time_exposed)
    Cd = 1.51;
    Wind_F = .5*Cd*rho*(595.394941/144)*Wind_S^2; %That 1291.1 is the side exposed surface area of the rocket THIS NEEDS TO BE CORRECTED FOR DIFFERENT ROCKETS
    time_exposed = time_exposed + delta_t;

end
