function [invSE3] = se3_inv(SE3)
% Function tranforms from a se(3) Lie Algebra to a SE(3) Lie Group
% using the Exponential Map.
%
% Inputs:
% SE3 = 4x4 Lie Algebra
%
% Outputs:
% invSE3 = 4x4 SE(3) Lie Group Inverse Matrix
%
% Date Created: 11/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% A Tutorial on SE(3) transformation parameterizations and on-manifold
% optimization - Jose Luis Blanco Claraco [2020]
%
% A Code for Unscented Kalman Filtering on Manifolds (UKF-M) - Brossard,
% Barrau, Bonnabel [2020]
%
% Nonparametric Second-order Theory of Error Propagation on Motion
% Groups - Wang, Chrikijian [2008]
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

% Separate State
R = SE3(1:3,1:3);
x = SE3(1:3,4);

invSE3 = [R' -R'*x;...
          0 0 0 1];