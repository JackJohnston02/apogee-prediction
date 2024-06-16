%% Script to generate measurements based off RocketPy Simulation Data

%% Set Sensor Sampling Rates
% We will calculate measurements at every timestep but in practice, sensors
% will generate measurements at different frequencies
sensor.gyro.fs = 100;
sensor.imu.fs = 100;
sensor.mag.fs = 100;
sensor.gnss.fs = 100;
sensor.alt.fs = 100;


% Change axes to align to standard
qrot = eul2quat([0,pi/2,0],"ZYX");

for i = 1:sim.numTimeSteps
    qNEW(:,i) = quatmultiply(qrot,sim.q(i,:))';
end

%% Gyroscope Measurements
% Gyroscope measures body angular rates which have low noise but bias.
sensor.gyro.biasRate = deg2rad(0);
sensor.gyro.biasNoiseSTD = 1e-4;
sensor.gyro.noiseSTD = 1e-3;
sensor.gyro.bias(1,:) = zeros(1,3);
for i = 2:sim.numTimeSteps
    sensor.gyro.bias(i,:) = sensor.gyro.bias(i-1,:) + sensor.gyro.biasRate*sim.Ts*ones(1,3);
end

% Generate Gyroscope Measurements
sensor.gyro.omega_b_meas = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    temp = sim.omega_b(i,:) + sensor.gyro.bias(i,:) + sensor.gyro.biasNoiseSTD*randn(1,3) + sensor.gyro.noiseSTD*randn(1,3);
    sensor.gyro.omega_b_meas(i,:) = [temp(3); temp(2); -temp(1)];
end

%% Accelerometer Measurements
% Accelerometer measures body accelerations which have high noise but no
% bias. (not true in practice but estimator will assume no bias)
sensor.imu.noiseSTD = 1e-2;
sensor.imu.biasNoiseSTD = 0;

% RocketPy accelerations are given in ENU coordinates so we must rotate
% into the body frame using the attitude quaternions
% Stationary accelerometer in ENU coordinates will measure a positive
% acceleration in Up (Z)
sim.a_b = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    if options.addGravityToAccelerationData
        sim.a_b(i,:) = quatrotate(quatconj(sim.q(i,:)),(sim.a_enu(i,:)+[0 0 9.81]));
    else
        sim.a_b(i,:) = quatrotate(quatconj(sim.q(i,:)),sim.a_enu(i,:));
    end
end

% Generate Accelerometer Measurements
sensor.imu.a_b_meas = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    sensor.imu.a_b_meas(i,:) = sim.a_b(i,:) + sensor.imu.noiseSTD*randn(1,3);
end

%% Magnetometer Measurements
% RocketPy does not provide Magnetic Field measurements so we need to
% calculate the true values based on the WMM2020 model
sensor.mag.noiseSTD = 1e-3;
sim.B_ned = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    sim.B_ned(i,:) = wrldmagm(sim.alt(i),sim.LLA(i,1),sim.LLA(i,2),decyear(2024,7,4),'2020');
end

% Convert NED Magnetic Field to ENU and rotate true values into body frame and add noise
sim.B_b = zeros(sim.numTimeSteps,3);
sim.B_enu = zeros(sim.numTimeSteps,3);
sensor.mag.B_b_meas = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    sim.B_enu(i,:) = [sim.B_ned(i,2) sim.B_ned(i,1) -sim.B_ned(i,3)];
    sim.B_b(i,:) = quatrotate(quatconj(sim.q(i,:)),sim.B_enu(i,:));
    % Normalise (only care about direction)
    sim.B_b(i,:) = sim.B_b(i,:)/norm(sim.B_b(i,:));
    sensor.mag.B_b_meas(i,:) = sim.B_b(i,:) + sensor.mag.noiseSTD*randn(1,3);
end

%% GNSS Measurements
% GNSS can provide position and velocity measurements, but to start only
% position will be used.
% GNSS provides measurements in LLA, which must be converted to XYZ
% Process is Geodetic (LLA) -> ECEF -> NED/ENU
% Altitude measurements are generally 1.5* less accurate than Lat/Long
sensor.gnss.noiseSTD = [1.5 1.5 1.5^2];
sim.wgs84 = wgs84Ellipsoid;
sensor.gnss.LLA_meas = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    % Use initial LLA as start point
    [xe,yn,zu] = geodetic2enu(sim.LLA(i,1),sim.LLA(i,2),sim.LLA(i,3),sim.LLA(1,1),sim.LLA(1,2),sim.LLA(1,3),sim.wgs84);
    tempPos = [xe,yn,zu] + randn(1,3)*blkdiag(sensor.gnss.noiseSTD(1),sensor.gnss.noiseSTD(2),sensor.gnss.noiseSTD(3));
     [lat,long,alt] = enu2geodetic(tempPos(1),tempPos(2),tempPos(3),sim.LLA(1,1),sim.LLA(1,2),sim.LLA(1,3),sim.wgs84);
     sensor.gnss.LLA_meas(i,:) = [lat,long,alt];
end

%% Altimeter Measurements
% Altitude provides Up/Down position measurements or altitude when using an
% ECEF or Geodetic frame
sensor.alt.noiseSTD = 0.01;
sensor.alt.h_meas = zeros(sim.numTimeSteps,1);
for i = 1:sim.numTimeSteps
    % Use initial LLA as start point
    sensor.alt.h_meas(i,1) = sim.alt(i,1) + sensor.alt.noiseSTD*randn;
end

%% Sample Measurements
sensor.imu.a_b_meas_samp = zeros(sim.numTimeSteps,3);
sensor.gyro.omega_b_meas_samp = zeros(sim.numTimeSteps,3);
sensor.mag.B_b_meas_samp = zeros(sim.numTimeSteps,3);
sensor.alt.h_meas_samp = zeros(sim.numTimeSteps,1);
sensor.gnss.LLA_meas_samp = zeros(sim.numTimeSteps,3);
for i = 1:sim.numTimeSteps
    % Timestep
    ts = i*sim.Ts;
    % Accelerometer
    if ~mod(i-1,round((1/sensor.imu.fs)/sim.Ts))
        sensor.imu.a_b_meas_samp(i,:) =  sensor.imu.a_b_meas(i,:);
    else
        sensor.imu.a_b_meas_samp(i,:) = [NaN NaN NaN];
    end
    % Gyroscope
    if ~mod(i-1,round((1/sensor.gyro.fs)/sim.Ts))
        sensor.gyro.omega_b_meas_samp(i,:) =  sensor.gyro.omega_b_meas(i,:);
    else
        sensor.gyro.omega_b_meas_samp(i,:) = [NaN NaN NaN];
    end
    % Magnetometer
    if ~mod(i-1,round((1/sensor.mag.fs)/sim.Ts))
        sensor.mag.B_b_meas_samp(i,:) =  sensor.mag.B_b_meas(i,:);
    else
        sensor.mag.B_b_meas_samp(i,:) = [NaN NaN NaN];
    end
    % GNSS
    if ~mod(i-1,round((1/sensor.gnss.fs)/sim.Ts))
        sensor.gnss.LLA_meas_samp(i,:) =  sensor.gnss.LLA_meas(i,:);
    else
        sensor.gnss.LLA_meas_samp(i,:) = [NaN NaN NaN];
    end
    % Altimeter
    if ~mod(i-1,round((1/sensor.alt.fs)/sim.Ts))
        sensor.alt.h_meas_samp(i,:) =  sensor.alt.h_meas(i,:);
    else
        sensor.alt.h_meas_samp(i,:) = NaN;
    end
end