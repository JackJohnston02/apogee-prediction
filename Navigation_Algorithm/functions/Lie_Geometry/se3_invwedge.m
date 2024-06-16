function [w] = se3_invwedge(wv)
% Function to perform the V (v) operation (Inverse Wedge) on a matrix.
%
% Inputs:
% wv = 3x3 matrix
%
% Outputs:
% w = 3x1 column vector
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

% Check Matrix Input
if ~ismatrix(wv)
    errordlg('Input to se3_wedge must be a Matrix.','Input Error');
    return
end

% Check Matrix Dimensions
[A,B] = size(wv);
if A ~= B || A ~= 3 || B ~= 3
    errordlg('Input to se3_wedge must be an 3x3 Matrix.','Input Error');
    return
end

w = [wv(3,2); wv(1,3); wv(2,1)];

end