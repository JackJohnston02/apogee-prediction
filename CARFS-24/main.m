%% Computational Aerodynamic Rocket Flight AirBrake Simulation
% This code simulates the real-time actuation of airbrakes in rocket flight.
%
% Based on the Computational Aerodynamic Rocket Flight Simulation (CARFS) code by Michael Blaser
% Adapted to include airbrake actuation by: Jack Johnston
%
% Date: May 3, 2024
% Copyright © 2024 Jack Johnston. All rights reserved.


%% %%%********** Todo *********%%%
%Include CARFS for supersonic simulations
%CARFS for density at altitude
%Include lookup tables for body Cd

%Add airbrakes to dynamics update, just add struct for airbrakes, Cd
%function and deployment variable
%Include 2d function for airbrakes Cd, mach no and deployment angle

% Add controller function, inputs should be all states, output should be
% motor velocity

%Add airbrake model, inputs should be motor velocity, output should be
%deployment angle lag, hard limits go here. 


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
Rocket.thrust = @(t) (t >= 0 & t < 0.1) .* (t * (thrust_data(1,2) / 0.1)) + ...
                     (t >= 0.1 & t <= Rocket.burntime) .* interp1(thrust_data(:,1), thrust_data(:,2), t, 'linear', 'extrap');

Rocket.mass = @(t) max(mass_wet - (mass_wet - mass_dry) * min(t, Rocket.burntime) / Rocket.burntime, mass_dry);


%% Import drag coeff data(body(on and off), airbrakes)

%% Define initial conditions
t = [0];
Rocket.x(1,1) = altitude_launch;
Rocket.x(1,2) = 0;
Rocket.x(1,3) = -9.81;

Rocket.state = "pad";


%% Main Loop
while Rocket.state ~= "landed"  && t(end) < 100
    t(end+1) = t(end) + dt;
    
    Rocket.state = state_update(Rocket);
    Rocket = dynamics_update(Rocket, t(end), dt);

end

%Comment out if dont want forced graphs
run("plot_data.m");

