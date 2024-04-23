%% Script to load RocketPy Simulation Data

% Load file
sim.data = readtable("Regulus 100.0Hz.csv");

% Extract vectors
sim.time = table2array(sim.data(:,1));
sim.Ts = sim.time(2,1) - sim.time(1,1);
sim.numTimeSteps = size(sim.time,1);
sim.x_enu = table2array(sim.data(:,2:4));
sim.alt = sim.x_enu(:,3);
sim.x_ned = [sim.x_enu(:,2) sim.x_enu(:,1) -sim.x_enu(:,3);];
sim.v_enu = table2array(sim.data(:,5:7));
sim.v_ned = [sim.v_enu(:,2) sim.v_enu(:,1) -sim.v_enu(:,3);];
sim.q = table2array(sim.data(:,8:11));
sim.omega_b = table2array(sim.data(:,12:14));
sim.LLA = [table2array(sim.data(:,15:16)) sim.alt];
sim.a_enu = table2array(sim.data(:,19:21));
sim.a_ned = [sim.a_enu(:,2) sim.a_enu(:,1) -sim.a_enu(:,3);];
sim.alpha_b = table2array(sim.data(:,22:24));
