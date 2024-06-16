function [wv] = so3_wedge(w)
% so3_wedge - Calculate the Skew-Symmetric matrix from a vector rotation
%
% Syntax:
%   [wv] = so3_wedge(w)
%
% Inputs:
%   w - [3x1] Skew Vector
%
% Output:
%   wv - [3x3] Skew-Symmetric Matrix
%
% Description:
%   Calulate the Skew-Symmetric matrix mapped to the corresponding 3D
%   vector.
%
% See also:
%   so3_invWedge
%
% References:
%
%   A Tutorial on SE(3) transformation parameterizations and on-manifold
%   optimization - Jose Luis Blanco Claraco [2020]
%
%   A Code for Unscented Kalman Filtering on Manifolds (UKF-M) - Brossard,
%   Barrau, Bonnabel [2020]
%
% Date Created: 12/04/2021
%
% Created by: Joe Gibbs
%
% Copyright (c) 2021 Joe Gibbs





% Check Length of Vector
if length(w) ~= 3
    errordlg('Input to so3_wedge must be a 3x1 Column Vector.','Dimension Error');
    return
end
    
wv = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

end