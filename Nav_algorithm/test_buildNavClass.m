% test script

% Random initial data
xi = [zeros(6,1); 1; zeros(6,1)];
Pt = eye(6);
Pa = eye(6);
q = 1e-3;

sigma.sigma_acc = 1e-3*eye(3);
sigma.sigma_alt = 1e-3*eye(3);
sigma.sigma_gyr = 1e-3*eye(3);
sigma.sigma_bias = 1e-3*eye(3);
sigma.sigma_gps = 1e-3*eye(3);
sigma.sigma_mag = 1e-3*eye(3);

settingsCA_KF.useGPSAltitude = false;
settingsMEKF.useAccUpdate = false;


nav = Navigation_Algorithm(xi,Pt,Pa,sigma,Ts,q,settingsCA_KF,settings_MEKF);