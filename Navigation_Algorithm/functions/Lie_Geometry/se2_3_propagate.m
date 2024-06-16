%% Test Script to Propagate SE2(3) Extended Pose
clear all;
clc;

% Initial Extended Pose
phi = 0;
theta = pi/4;
psi = pi/8;

Ri = angle2dcm(phi,theta,psi);
xi = [0; 0; 0];
vi = [0; 0; 0];

posei = [Ri vi xi;...
         zeros(1,3) 1 0;...
         zeros(1,3) 0 1];

% Propagation [omega, v, p]
w = [0.5; -0.4; 0; 0; 0; 0; 0.5; 0.5; 0];
dt = 0.1;

pose{1} = posei;
se3{1} = [Ri xi; zeros(1,3) 1];
% Equivalent SE(3) lie algebra components are the rotation and position
% elements (1:3) and (7:9)
ws = [w(7:9); w(1:3)];
vp{1} = w(1:3);
% Run Test Sim
for i = 2:1000
    pose{i} = pose{i-1}*se2_3_expmap(w*dt);
    %% SE3 Equivalent
    se3{i} = se3{i-1}*se3_expmap(ws*dt);
end

% Extract Position
for i = 1:1000
    pos(:,i) = pose{i}(1:3,5);
    vel(:,i) = pose{i}(1:3,4);
    se3pos(:,i) = se3{i}(1:3,4);
end

figure;
hold on;
plot3(pos(1,:),pos(2,:),pos(3,:),'color','b','DisplayName','SE_2(3) Position');
plot3(se3pos(1,:),se3pos(2,:),se3pos(3,:),'color','r','DisplayName','SE(3) Position');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend;
grid minor;