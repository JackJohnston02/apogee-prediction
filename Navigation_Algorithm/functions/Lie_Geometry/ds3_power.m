function [DQp] = ds3_power(DQ,n)
% Function to calculate the nth power of a unit dual quaternion.
%
% Inputs:
% DQ - [8x1] Pure Dual Quaternion Lie Algebra
% n  - [1x1] Power
%
% Outputs:
% DQp - [8x1] Unit Dual Quaternion Power
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

% Quaternion Multiplication requires row vectors

% Check for formatting
if height(DQ) > 1
    DQ = DQ';
end

% Calculate power of unit dual quaternion using log and exponential map
DQp = ds3_expmap(n*ds3_logmap(DQ));
end



