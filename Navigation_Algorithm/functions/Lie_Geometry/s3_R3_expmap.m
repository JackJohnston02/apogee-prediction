function [X] = s3_R3_expmap(x)
% Function to tranform from a SO(3)xR3 Tangent Space to a S3xR3 Manifold
% using the Exponential Map of S3.
%
% Inputs:
% wv - Rotation Vector Element of se(3)
%
% Outputs:
% q - S3 Manifold ELement
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
%   To remove the ambiguity about the direction, the argument of arccos is
%   checked for non-negativity.

% Separate Components
% Rotation
wv = x(1:3);
% Angular Velocity
w = x(4:6);

% S3 Exponential Map
q = s3_expmap(wv);

% Combine
X = [q; w];

end
