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

rng('default');

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
X0 = [quat2dcm(sim.qned(:,1)') zeros(3,1) zeros(3,1); 0 0 0 1 0; 0 0 0 0 1];
Z0 = zeros(6,1);
P0 = blkdiag(0.1*eye(3),0.01*eye(3),1*eye(3),0.001^2*eye(3),0.0001^2*eye(3));

settingsKF.useGNSSAltitude = true;
settingsKF.propagateWithAccelerometer = true;
%% USING ACCELEROMETER UPDATE CAUSES ISSUES
settingsKF.useAccUpdate = false;
settingsKF.useAccUpdate = false;
settingsApPred.usePredict = true;
settingsApPred.numParticles = 1;

sigma.sigma_acc = sensor.imu.noiseSTD;
sigma.sigma_accBias = sensor.imu.biasNoiseSTD;
sigma.sigma_alt = sensor.alt.noiseSTD;
sigma.sigma_gyr = sensor.gyro.noiseSTD;
sigma.sigma_bias = sensor.gyro.biasNoiseSTD;
sigma.sigma_gnss = sensor.gnss.noiseSTD;
sigma.sigma_mag = sensor.mag.noiseSTD;

Qgyro = blkdiag(eye(3)*sensor.gyro.noiseSTD);
Qgyrob = blkdiag(eye(3)*sensor.gyro.biasNoiseSTD);
Qacc = blkdiag(eye(3)*sensor.imu.noiseSTD);
Qaccb = blkdiag(eye(3)*sensor.imu.biasNoiseSTD);
Rgnss = diag(sensor.gnss.noiseSTD);
Ralt = blkdiag(sensor.alt.noiseSTD);
Rmag = blkdiag(eye(3)*sensor.mag.noiseSTD);


% Instatitate Navigation Algorithm
nav = Navigation_Algorithm_Single(X0,Z0,P0,Qgyro,Qgyrob,Qacc,Qaccb,Rgnss,Ralt,Rmag,sim.Ts,settingsKF,settingsApPred,sim.LLA(1,:));

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
    Xest{i} = nav.estimator.Xkk;
    Zest(:,i) = nav.estimator.Zkk;
    % west(:,i) = nav.omega;
    apest(1,i) = nav.h_ap;

end

% Convert NED position to ENU and LLA for plotting
for i = 1:sim.numTimeSteps
    x_estNED(i,:) = Xest{i}(1:3,5); 
    v_estNED(i,:) = Xest{i}(1:3,4); 
    % a_estNED(i,:) = [xestT(3,i) xestT(6,i) xestT(9,i)];
end


% Attitude using NED reference frame
% Rotate reference vector for 3D plot
% e3 = [0 0 1];
% for i = 1:size(sim.time,1)
%     s3est(i,:) = quatrotate(xestA(1:4,i)',e3);
% end



% Plot Data
if options.plotData
    plotRocketPyData;
end

if settingsApPred.usePredict
    plotApogeePrediction;
end