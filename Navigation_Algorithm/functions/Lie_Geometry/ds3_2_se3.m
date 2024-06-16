function [X] = ds3_2_se3(DQ)
% Function to transform a [8x1] Unit Dual Quaternion in DS^3
% to a [4x4] Homegenous Transformation Matrix in SE(3)
%
% Inputs:
% DQ - [8x1] Unit Dual Quaternion in DS^3 Lie Group
%
% Outputs:
% X - [4x4] Homogenous Transformation Matrx in SE(3) Lie Group
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

% Extract Real and Dual Parts
r = DQ(1:4);
d = DQ(5:8);

% Form Rotation Matrix from Real Quaternion
R = quat2rotm(r);

% Calculate Translation Vector
t = 2*quatmultiply(d,quatconj(r))';

% Form Homegenous Transformation Matrix
X = [R t(2:4); 0 0 0 1];

end