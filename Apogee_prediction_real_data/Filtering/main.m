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

flightname = "owen";
filename = 'data/' + flightname + '.csv';




%% Define all the filters here, need to add filter names to the initialise filter switchcase in run_filter.m
filters = ["UKF_constant_acceleration", "UKF_constant_Cb", "EKF_constant_acceleration", "EKF_constant_Cb", "CKF_constant_acceleration", "CKF_constant_Cb", "All"];

%% Filter parameters
sigma_Q = 0.2; % 0.2
sigma_Q_Cb = 2; % 25 for owen.csv, 2 for regulus.csv

initial_covariance = 1 * [1, 0, 0, 0;
                            0, 1, 0, 0;
                            0, 0, 1, 0;
                            0, 0, 0, 10e2];
% For optimal results 
    % CCb UKF:
        % sigma_Q = 0.1
        % sigma_Q_Cb = 10
    % CCb EKF:
        % sigma_Q = 
        % sigma_Q_Cb = 


% 24.5ish for regulus7
% 18ish for owen
measurement_sigma_bar = 0.5744578867366569; % Barometer measurement STD
measurement_sigma_acc = 0.006942717204787825; % Accelerometer measurement STD

if filename == "data/owen.csv"
    burnout_time = 9;
    apogee_time = 17.65;

elseif filename == "data/regulus.csv"
    apogee_time = 27;
    burnout_time = 3;
end

for i = 1:length(filters)
    disp(i + " for " + filters(i))
end

filter_choice = filters(input("Choose Filter:"));

% Timing
filterTimes = struct();

if filter_choice == "All"
    for i = 1:length(filters)-1
        filter_type = filters(i);

        filterTimes.(filter_type).timeUpdateTimes = [];
        filterTimes.(filter_type).measurementUpdateTimes = [];
        filterTimes.(filter_type).apogeePredictionTimes = [];
    
        % Initialize the average times for each step in the nested struct
        filterTimes.(filter_type).avgTimeUpdate = 0;
        filterTimes.(filter_type).avgMeasurementUpdate = 0;
        filterTimes.(filter_type).avgApogeePrediction = 0; 
        
        run("run_filter.m");
    end
    run("comparison_plotting.m")
else
    filter_type = filter_choice;
    run("run_filter.m")
end

disp("Finished")
