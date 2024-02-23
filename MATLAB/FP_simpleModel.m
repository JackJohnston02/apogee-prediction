function [est_ap_time,est_ap_alt] = FP_simpleModel(x_future, t_future, dt)
%FORWARDPROPAGATION nothign fancy, just some SUVAT


    A = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1]; %State transition matrix
    while x_future(2) > 0 
        x_future = A*x_future;
        t_future = t_future + dt;
    end
    est_ap_time = t_future;
    est_ap_alt = x_future(1);
end

