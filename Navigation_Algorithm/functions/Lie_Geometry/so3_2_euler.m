function [Eul] = so3_2_euler(SO3)
% Function to transform a SO(3) Rotation Matrix to equivalent Euler Angles.
%
% Inputs:
% SO3 = 3x3 Rotation Matrix in the Group SO(3)
%
% Outputs:
% Eul = 3x1 Column Vector of Roll, Pitch and Yaw angles
%
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

% Define Rotation Matrix as R for simplicity
R = SO3;

% Calculate Pitch Angle (Theta)
theta = atan2(-R(3,1), sqrt(R(1,2)^2 + R(2,1)^2));

% Condition if theta is approaching +90deg
if abs(theta - pi/2) < 1e-9
    psi = 0;
    phi = atan2(R(1,2), R(2,2));
    
% Condition is theta is approachin -90deg
elseif abs(theta + pi/2) < 1e-9
    psi = 0;
    phi = -atan2(R(1,2), R(2,2));

% Otherwise
else
    psi = atan2(R(2,1)/cos(theta), R(1,1)/cos(theta));
    phi = atan2(R(3,2)/cos(theta), R(3,3)/cos(theta));
end

% Otherwise


