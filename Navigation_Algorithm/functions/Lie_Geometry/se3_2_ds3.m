function [DQ] = se3_2_ds3(X)
% Function to transform a [4x4] Homegenous Transformation Matrix in SE(3)
% to a [8x1] Unit Dual Quaternion in DS^3
%
% Inputs:
% X - [4x4] Homogenous Transformation Matrx in SE(3) Lie Group
%
% Outputs:
% DQ - [8x1] Unit Dual Quaternion in DS^3 Lie Group
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

% Extract Rotational and Translational Components
R = X(1:3,1:3);
t = X(1:3,4);

% Convert Rotation Matrix to Unit Quaternion
qr = rotm2quat(R);

% Append Translation Vector
qt = [0; t];

% Form Unit Dual Quaternion
DQ = [qr 0.5*quatmultiply(qt',qr)]';

end