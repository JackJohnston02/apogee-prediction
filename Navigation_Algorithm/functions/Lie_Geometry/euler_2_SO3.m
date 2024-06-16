function [SO3] = euler_2_SO3(Eul)
% Function to transform a set of Euler Angles to the 3x3 Rotation Matrix
% in SO(3).
%
% Inputs:
% Eul = 3x1 Column Vector of Roll, Pitch and Yaw angles
%
% Outputs:
% SO3 = 3x3 Rotation Matrix in the Group SO(3)
%
% Date Created: 11/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Computing Euler Angles from a Rotation Matrix - Slabaugh
%
% https://stackoverflow.com/questions/15022630/how-to-calculate-
% the-angle-from-rotation-matrix

% Separate Euler Angles
phi = Eul(1);
theta = Eul(2);
psi = Eul(3);

% Rotation in x
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
% Rotation in y
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
% Rotation in z
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

% Combined Rotation
R = Rz*Ry*Rx;

SO3 = R;

end
