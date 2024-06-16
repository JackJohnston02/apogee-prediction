function [DQs] = ds3_sclerp(DQ1,DQ2,f)
% Function to perform the Screw Linear Interpolation between two unit dual
% quaternions.
%
% Inputs:
% DQ1 - [8x1] Unit Dual Quaternion
% DQ2 - [8x1] Unit Dual Quaternion
% f   - [1x1] Interval Fraction
%
% Outputs:
% DQs - [8x1] ScLERP Unit Dual Quaternion
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

% Calculate ScLERP
DQs = ds3_product(dq1,ds3_power(ds3_product(ds3_conjugate(dq1),dq2),f));
end
