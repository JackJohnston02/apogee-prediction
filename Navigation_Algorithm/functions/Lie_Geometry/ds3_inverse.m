function [DQinv] = ds3_inverse(DQ)
% Function to calculate the inverse of a dual quaternion.
%
% Inputs:
% DQ - [8x1] Pure Dual Quaternion
%
% Outputs:
% DQinv - [8x1] Inverse of Unit Dual Quaternion Product
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

% Check for formatting
if height(DQ) > 1
    DQ = DQ';
end

% Extract S3 Quaternions
d = DQ(1:4);
r = DQ(5:8);

% Check for Zero-norm condition
if abs(quatnorm(d'))
    error('Quaternion has zero-norm, non-invertible.');
end

% Calculate Dual Quaternion Inverse
DQinv = [quatinv(d) -quatmultiply(quatinv(d),quatmultiply(r,quatinv(d)))]';

end


