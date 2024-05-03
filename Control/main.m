%% Code for real-time airbrake actuation simulations 
% Programmer(s): Jack Johnston
% copyright © 2024


%% %%%********** Todo *********%%%
%Include CARFS for supersonic simulations
%CARFS for density at altitude

%% %%%*********MAINSCRIPT*********%%%
clc
clear 

%% Simulation Settings
rocket_file_name = "Regulus";%File containing rocket data
dt = 0.01; %Simulation timestep


%% Import Data
%Rocket data
run(rocket_file_name);
fprintf(rocket_file_name + " data used for simulation")


%Motor data
file_path = fullfile(motor_path);
% Open the file
fileID = fopen(file_path, 'r');

% Check if file exists
if fileID == -1
    error(motor_path+' not found');
end

% Read the file
thrust_data = textscan(fileID, '%f %f', 'HeaderLines', 1);

% Close the file
fclose(fileID);

% Check if data is empty
if isempty(thrust_data{1}) || isempty(thrust_data{2})
    error('No data in ' + motor_file_name);
end

% Convert cell array to matrix
thrust_data = cell2mat(thrust_data);

%Functions for mass thrust
Rocket.burntime = max(thrust_data(:,1));
Rocket.thrust = @(t) (t >= 0 & t <= Rocket.burntime) .* interp1(thrust_data(:,1), thrust_data(:,2), t, 'linear', 'extrap');
Rocket.mass = @(t) max(mass_wet - (mass_wet - mass_dry) * min(t, Rocket.burntime) / Rocket.burntime, mass_dry);

%% Define initial conditions
t = 0;
Rocket.x(1,1) = altitude_launch;
Rocket.x(1,2) = 0;
Rocket.x(1,3) = -9.81;

Rocket.state = "pad";


%% Main Loop
while Rocket.state ~= "landed"  && t < 100
    t = t + dt;

    Rocket.state = state_update(Rocket);
    Rocket = dynamics_update(Rocket, t, dt);

end
