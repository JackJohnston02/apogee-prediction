function [Jl_so2] = so2_leftJacobian(so2)
% Function to calculate the Left Jacobian Matrix of a SO(2) Lie Algebra or
% Lie Group.
%
% Inputs:
% so2 = 1x1 Lie Algebra
%
% Outputs:
% Jl_so2 = 2x2 Left Jacobian Matrix of SO(2)
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

% Define Lie Algebra as Rotation Vector
w = so2;

% Calulate Wedge Operation
wv = so2_wedge(w);

% Condition for theta~0, use a First Order Taylor Series Expansion
if abs(theta) < 1e-9
    Jl_so2 = eye(2) + wv/2;
else
    Jl_so2 = eye(2)*(sin(w)/w) + so2_wedge(1)*(1 - cos(w))/w;
end

end
