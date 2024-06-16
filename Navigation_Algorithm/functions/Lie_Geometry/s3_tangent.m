function [T] = s3_tangent(q)
% Function to extract the tangent vector from a unit quaternion
% representing attitude in S^3;
%
% Inputs:
% q - [4x1] Unit Quaternion in S3
%
% Outputs:
% T - [3x1] Tangent Vector in R3
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

% Define Tangent as Positive X-Axis
e1 = [1; 0; 0];

% Reshape for Quaternion Multiplication
e1q = [0; e1]';

% Perform Quaternion Rotation
Tq = quatmultiply(q',quatmultiply(e1q,quatconj(q')))';

% Select Vector Components
T = Tq(2:4)';

end


