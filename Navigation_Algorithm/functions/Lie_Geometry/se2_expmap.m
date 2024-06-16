function [SE2] = se2_expmap(se2)
% Function to tranform from a se(2) Lie Algebra to a SE(2) Lie Group
% using the Exponential Map.
%
% Inputs:
% se2 = 3x1 Lie Algebra
%
% Outputs:
% SE2 = 3x3 SE(2) Lie Group
%
% Date Created: 12/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

w = se2(1);

% Rotational Component
R = [cos(w) sin(w); sin(w) cos(w)];

% Use SO2 Jacobian to aovid errors with theta ~0
a = so2_leftJacobian(w) * se2(2:3);

% Generate SE(2) Lie Group
SE2 = [R a; 0 0 1];

end