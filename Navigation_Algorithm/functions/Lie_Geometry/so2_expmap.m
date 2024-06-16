function [SO2] = so2_expmap(so2)
% Function to tranform from a so(2) Lie Algebra to a SO(2) Lie Group
% using the Exponential Map.
%
% Inputs:
% so2 = 1x1 Lie Algebra
%
% Outputs:
% SO2 = 2x2 SO(2) Lie Group Rotation Matrix
%
% Date Created: 12/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

% Define Lie Algebra as Rotation Vector
w = so2;

% Calculate Lie Group
SO2 = [cos(w) sin(w); sin(w) cos(w)];

end