%% Script to run a simulation of the Navigation Algorithm based on RocketPy\
%% simulation data
% Author - Joe Gibbs
% Date   - 07/04/2024

% Housekeeping
clc;
clear all;
close all;
all_fig = findall(0, 'type', 'figure');
close(all_fig)

%% OPTIONS
options.plotData = true;
options.addGravityToAccelerationData = true;

% Load RocketPy Simulation Data
loadRocketPyData;

% Generate NED Quaternions
qenu2ned = [0 0.5*sqrt(2) 0.5*sqrt(2) 0];
for i = 1:sim.numTimeSteps
    sim.qned(:,i) = quatmultiply(qenu2ned,sim.q(i,:))';
end

% Generate Sensor Measurements
generateSensorMeasurements;

% Initial Values for Navigation Algorithm
% Random initial data
qi = sim.qned(:,1);
xi = [zeros(1,9) qi' zeros(1,3)]';
Pt = eye(9);
Pa = blkdiag(0.1,0.1,0.1,0.1,0.1,0.1);
q = 100;

settingsCA_KF.useGNSSAltitude = false;
settingsCA_KF.propagateWithAccelerometer = false;
%% USING ACCELEROMETER UPDATE CAUSES ISSUES
settingsCA_KF.useAccUpdate = true;
settingsMEKF.useAccUpdate = false;
settingsApPred.usePredict = true;
settingsApPred.numParticles = 1;

sigma.sigma_acc = sensor.imu.noiseSTD;
sigma.sigma_alt = sensor.alt.noiseSTD;
sigma.sigma_gyr = sensor.gyro.noiseSTD;
sigma.sigma_bias = sensor.gyro.biasNoiseSTD;
sigma.sigma_gnss = sensor.gnss.noiseSTD;
sigma.sigma_mag = sensor.mag.noiseSTD;


% Instatitate Navigation Algorithm
nav = Navigation_Algorithm(xi,Pt,Pa,sigma,sim.Ts,q,settingsCA_KF,settingsMEKF,settingsApPred,sim.LLA(1,:));

%% Run Simulation
for i = 1:sim.numTimeSteps

    % Add latest measurements to the buffer. If a measurement is not available
    % a NaN value will be in it's place
    buffer.y_acc = sensor.imu.a_b_meas_samp(i,:)';
    buffer.y_omega = sensor.gyro.omega_b_meas_samp(i,:)';
    buffer.y_gnss = sensor.gnss.LLA_meas_samp(i,:)';
    buffer.y_mag = sensor.mag.B_b_meas_samp(i,:)';
    buffer.y_alt = sensor.alt.h_meas_samp(i,:)';

    %% FEED IN TRUE DATA FOR APOGEE PREDICTION
    nav.trueU = [sim.x_enu(i,3); sim.v_enu(i,3); sim.a_enu(i,3)];
    if i >= 298
        testAp(1,i) = FP_Model(nav.trueU',eye(3),[],0.01);
    else
        testAp(1,i) = 0;
    end


    % Implement iteration of Navigation Algorithm
    nav = nav.updateNavigationStates(buffer,sim.qned(:,i));


    % Store State Estimates for Plotting
    xestT(:,i) = nav.filters.translationKF.xkk;
    xestA(:,i) = nav.filters.attitudeKF.xkk;
    west(:,i) = nav.filters.attitudeKF.omega;
    apest(1,i) = nav.h_ap;

end

% Convert NED position to ENU and LLA for plotting
for i = 1:sim.numTimeSteps
    x_estNED(i,:) = [xestT(1,i) xestT(4,i) xestT(7,i)]; 
    v_estNED(i,:) = [xestT(2,i) xestT(5,i) xestT(8,i)];
    a_estNED(i,:) = [xestT(3,i) xestT(6,i) xestT(9,i)];
end


% Attitude using NED reference frame
% Rotate reference vector for 3D plot
e3 = [0 0 1];
for i = 1:size(sim.time,1)
    s3est(i,:) = quatrotate(xestA(1:4,i)',e3);
end



% Plot Data
if options.plotData
    plotRocketPyData;
end

if settingsApPred.usePredict
    plotApogeePrediction;
end