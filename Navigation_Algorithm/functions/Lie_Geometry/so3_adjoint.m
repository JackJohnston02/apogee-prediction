function [adjSO3] = so3_adjoint(SO3)
% Function to compute the Adjoint of SO(3).
%
% Inputs:
% SO3 = 3x3 Rotation Matrix in the Group SO(3)
%
% Outputs:
% adjSO3 = 3x3 Matrix
%
% Date Created: 11/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

adjSO3 = SO3;
