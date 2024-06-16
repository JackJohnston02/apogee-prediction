function [se3] = se3_logmap(SE3)
% Function tranforms from a se(3) Lie Algebra to a SE(3) Lie Group
% using the Exponential Map.
%
% Inputs:
% SE3 = 4x4 SE(3) Lie Group
%
% Outputs:
% se3 = 6x1 Lie Algebra
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
%
% Nonparametric Second-order Theory of Error Propagation on Motion
% Groups - Wang, Chrikijian [2008]
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]


% Split the Lie Group into Components
exp_wv = SE3(1:3,1:3);

% Check Matrix Input
if ~ismatrix(SE3)
    errordlg('Input to so3_logmap must be a Matrix.','Input Error');
    return
end

% Check Matrix Dimensions
[A,B] = size(SE3);
if A ~= B || A ~= 4 || B ~= 4
    errordlg('Input to se3_logmap must be an 4x4 Matrix.','Input Error');
    return
end


% Calculate angle of Rotation (theta)
theta = acos((trace(exp_wv)-1)/2);

% Logarithm of Rotation
ln_exp_wv = (theta/(2*sin(theta)))*(exp_wv - exp_wv');

% Rotation Element of Lie Algebra
w = [-ln_exp_wv(2,3) ln_exp_wv(1,3) -ln_exp_wv(1,2)];

% Wedge Operation (w^) of (w) maps (w) to it's skew symmetric matrix
wv = se3_wedge(w);

% Condition for Zero Rotation
if theta == 0
    V = eye(3);
else
    V = eye(3) - (0.5)*wv + (1/(theta^2))*(1-((sin(theta)/theta)/(2*((1-cos(theta))/(theta^2)))))*wv^2;
end

Vt = SE3(1:3,4);

% Calculate Rotation Components
t = V*Vt;

% Output Lie Algebra
se3 = [t' w];

end