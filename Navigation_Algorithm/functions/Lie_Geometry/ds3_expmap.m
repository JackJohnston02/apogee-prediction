function [DQX] = ds3_expmap(DQ)
% Function to tranform from a DS^3 Lie Algebra to a DS^3 Lie Group
% using the Exponential Map of the dual quaternion Lie Group.
%
% Inputs:
% DQ - [8x1] Pure Dual Quaternion Lie Algebra
%
% Outputs:
% DQX - [8x1] Unit Dual Quaternion
%
% Date Created: 01/02/2022
%
% Created by: Joe Gibbs
%
% References:
%
% A Tutorial on SE(3) transformation parameterizations and on-manifold
% optimization - Jose Luis Blanco Claraco [2020]
%
% A Micro Lie Theory for State Estimation in Robotics - Sola, Deray,
% Atchuthan [2020]
%
% Quaternion Kinematics of the Error-State Kalman Filter - Sola [2012]
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]
%
% http://jamessjackson.com/lie_algebra_tutorial/06-closed_form_mat_exp/
%
% https://uk.mathworks.com/matlabcentral/fileexchange/71397-robotics-dual-quaternion-toolbox

% Quaternions must be Pure Quaternions, i.e. qw0, qv0 ~0

% Extract elements for ease of reading (wv = [qw; qv])
w = DQ(2:4);
v = DQ(6:8);

% Calculate Rotation Angle
theta = norm(w);

% Check for small/no rotation using float precision as tolerance
if abs(theta) < eps
    DQX = [1; 0; 0; 0; 0; v];
    return;
end

% Unit Rotation Axis Element
wu = w/theta;

% Screw Angle
y = 2*wu'*v;

% Coefficients for ease of reading
a = (v - 0.5*y*wu)/theta;
b = 0.5*y*cos(theta);

% Retract onto Lie Group
DQw = [cos(theta); sin(theta)*wu];
DQv = [-0.5*y*sin(theta); a*sin(theta) + b*wu];

DQX = [DQw; DQv];

end


