function [so2] = so2_logmap(SO2)
% Function to tranform from a SO(2) Lie Group to a SO(2) Lie Algebra
% using the Logarithmic Map.
%
% Inputs:
% SO2 = 2x2 SO(2) Lie Group Rotation Matrix
%
% Outputs:
% so2 = 1x1 Lie Algebra
%
% Date Created: 12/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]


% Calculate Lie Algebra
so2 = atan2(SO2(2,1),SO2(1,1));

end