%% TODO
% Add EKF 
    % CCb
    % Ca
% Add Cubature
    % CCb
    % Ca

% Add iterative script for all filters
% Add comparison plotting for all filters
% Remove the subsequent values from the ZOH sensor data
  

%% Tidy up
clear all;
close all; % Uncomment if comparing plots
addpath("filters\")
filename = 'data/owen.csv';

%% Select filter type
% UKF_constant_acceleration
% UKF_constant_Cb

%% Define all the filters here, need to add filter names to the initialise filter switchcase in run_filter.m
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", "All"];

for i = 1:length(filters)
    disp(i + " for " + filters(i))
end

filter_choice = filters(input("Choose Filter:"));


if filter_choice == "All"
    for i = 1:length(filters)-1
        filter_type = filters(i);
        run("run_filter.m");
    end
else
    filter_type = filter_choice;
    run("run_filter.m")
end


disp("Finished")
