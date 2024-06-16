function [SO3] = so3_expmap(so3)
% Function to tranform from a so(3) Lie Algebra to a SO(3) Lie Group
% using the Exponential Map.
%
% Inputs:
% so3 = 3x1 Lie Algebra
%
% Outputs:
% SO3 = 3x3 SO(3) Lie Group Rotation Matrix
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

% Define Lie Algebra as Rotation Vector
w = so3;

% Calculate angle of Rotation (theta)
theta = norm(w);

% Condition for theta ~0, use a First Order Taylor Series Expansion
if abs(theta) < 1e-9
    exp_wv = eye(length(so3)) + so3_wedge(w);
    % Otherwise
else
    % Axis Components
    axis = w / theta;
    exp_wv = cos(theta)*eye(length(so3)) + (1 - cos(theta))*(axis*axis') +...
        sin(theta)*so3_wedge(axis);
end

SO3 = exp_wv;

end

