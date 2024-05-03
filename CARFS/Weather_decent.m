function [Wind_F,time_exposed_decent] = Weather_decent(rho,Wind_S, delta_t,time_exposed_decent)
    Cd = 1.51;
    Wind_F = .5*Cd*rho*(595.394941/144)*Wind_S^2;
    time_exposed_decent = time_exposed_decent + delta_t;
    
    
end
