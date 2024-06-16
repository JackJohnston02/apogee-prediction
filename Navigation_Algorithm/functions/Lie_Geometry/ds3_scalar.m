function [s] = ds3_scalar(DQ)
% Function to extract the scalar of the dual quaternion.
%
% Inputs:
% DQ - [8x1] Unit Dual Quaternion
%
% Outputs:
% s - [8x1] Unit Dual Quaternion Scalar
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

% Extract Dual Quaternion Scalar
s = DQ(1);

end

