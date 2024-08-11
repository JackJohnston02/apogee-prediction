4%% TODO
% Add EKF 
    % CCb
    % Ca
% Add Cubature
    % CCb
    % Ca

% Add iterative script for all filters
% Add comparison plotting for all filters
% Remove the subsequent values from the ZOH sensor data
  

%%3 Tidy up
clear all;
close all; % Uncomment if comparing plots
addpath("filters\")
filename = 'data/owen.csv';

%% Select filter type
% UKF_constant_acceleration
% UKF_constant_Cb

%% Define all the filters here, need to add filter names to the initialise filter switchcase in run_filter.m
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", "EKF_constant_acceleration", "EKF_constant_Cb", "All"];

%% Filter parameters
sigma_Q = 0.1; %0.1
sigma_Q_Cb = 100; %100
measurement_noise_bar = 0.5744578867366569;
measurement_noise_acc = 0.006942717204787825;

for i = 1:length(filters)
    disp(i + " for " + filters(i))
end

filter_choice = filters(input("Choose Filter:"));


if filter_choice == "All"
    for i = 1:length(filters)-1
        filter_type = filters(i);
        run("run_filter.m");
    end
    run("comparison_plotting.m")
else
    filter_type = filter_choice;
    run("run_filter.m")
end

disp("Finished")
