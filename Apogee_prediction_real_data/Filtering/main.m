% TODO
% Add particle filter
% Add Cubature
    % CCb
    % Ca

% Remove the subsequent values from the ZOH sensor data - might have done
% this?
  

%%3 Tidy up
clear all;
close all; % Uncomment if comparing plots
addpath("filters\")
filename = 'data/owen.csv';


%% Define all the filters here, need to add filter names to the initialise filter switchcase in run_filter.m
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", "EKF_constant_acceleration", "EKF_constant_Cb", "CKF_constant_acceleration", "CKF_constant_Cb", "All"];

%% Filter parameters
sigma_Q = 0.1;
sigma_Q_Cb = 100;
% For optimal results 
    % CCb UKF:
        % sigma_Q = 0.1
        % sigma_Q_Cb = 100
    % CCb EKF:
        % sigma_Q = 
        % sigma_Q_Cb = 

measurement_sigma_bar = 0.5744578867366569; % Barometer measurement STD
measurement_sigma_acc = 0.006942717204787825; % Accelerometer measurement STD

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
