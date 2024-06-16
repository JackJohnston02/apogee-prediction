function [LB] = lieBrackets(A, B)
% Function to perform the Lie Brackets [A,B] operation.
%
% Inputs:
% Smooth Vector Fields (A) and (B)
%
% Outputs:
%
%
% Functions:
%
% Date Created: 03/11/2020

LB = A*B - B*A;
