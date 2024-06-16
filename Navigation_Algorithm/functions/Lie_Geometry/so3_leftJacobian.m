function [Jl_so3] = so3_leftJacobian(so3)
% Function to calculate the Left Jacobian Matrix of a SO(3) Lie Algebra or
% Lie Group.
%
% Inputs:
% so3 = 3x1 Lie Algebra
%
% Outputs:
% Jl_so3 = 3x3 Left Jacobian Matrix of SO(3)
%
% Date Created: 12/11/2020
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
%
% Vision Based Navigation Course Notes - Usenko, Demmel, Schubert, 
% Cremers [2020]

% Transform the input if required
if isrow(so3)
    so3 = so3';
end

if length(so3) == 3
    % Define Lie Algebra as Rotation Vector
    w = so3;
else
    errordlg('Input to so3_leftJacobian has incorrect dimensions.','Dimension Error');
    return
end
% Calculate angle of Rotation (theta)
theta = norm(w);

% Calulate Wedge Operation
wv = so3_wedge(w);

% Condition for theta~0, use a First Order Taylor Series Expansion
if abs(theta) < 1e-9
    Jl_so3 = eye(length(so3)) - wv/2;
else
    alpha = w / theta;
    Jl_so3 = eye(length(so3))*(sin(theta)/theta) + (1 - sin(theta))/theta*...
        (alpha*alpha') + (1 - cos(theta))/theta*so3_wedge(alpha);
end

end
    