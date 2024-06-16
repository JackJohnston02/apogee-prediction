function [se2] = se2_logmap(SE2)
% Function to tranform from a se(2) Lie Algebra to a SE(2) Lie Group
% using the Exponential Map.
%
% Inputs:
% SE2 = 3x3 SE(2) Lie Group
%
% Outputs:
% se2 = 3x1 Lie Algebra
% 
% Date Created: 12/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]


% Rotational Component
R = SE2(1:2,1:2);
so2 = atan2(R(2,1),R(1,1));

% Translational Component
% Use SO2 Jacobian to aovid errors with theta ~0
a = so2_leftInvJacobian(so2) * SE2(1:2,3);

% Calculate Lie Algebra
se2 = [so2; a];

end