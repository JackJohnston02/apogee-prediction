

%% Initialise filter
X0 = [eye(3) zeros(3,1) zeros(3,1); 0 0 0 1 0; 0 0 0 0 1];
Z0 = zeros(6,1);
P0 = eye(15);

Qgyro = blkdiag(0.01^2*eye(3));
Qgyrob = blkdiag(0.05^2*eye(3));
Qacc = blkdiag(0.005^2*eye(3));
Qaccb = blkdiag(0.005^2*eye(3));
Rgnss = blkdiag(0.5^2,0.5^2,1^2);
Ralt = blkdiag(0.25^2);
Rmag = blkdiag(0.01*eye(3));
dt = 0.01;

riekf = RIEKF(X0,Z0,P0,Qgyro,Qgyrob,Qacc,Qaccb,Rgnss,Ralt,Rmag,dt);