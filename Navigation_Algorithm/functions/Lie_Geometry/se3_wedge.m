function [wv] = se3_wedge(w)
% Function to perform the Wedge (^) operation on a vector.
%
% Inputs:
% w = 3x1 column vector
%
% Outputs:
% wv = 3x3 matrix
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


% Check Length of Vector
if length(w) ~= 3
    errordlg('Input to se3_wedge must be a 3x1 Column Vector.','Dimension Error');
    return
end

wv = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

end