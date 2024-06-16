function [so3] = so3_logmap(SO3)
% Function tranform from a SO(3) Lie Group to a so(3) Lie Algebra
% using the Logarithmic Map.
%
% Inputs:
% SO3 = 3x3 SO(3) Lie Group Rotation Matrix
%
% Outputs:
% so3 = 3x1 Lie Algebra
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


% Define Lie Group as Rotation Matrix
exp_wv = SO3;

% Check Matrix Input
if ~ismatrix(exp_wv)
    errordlg('Input to so3_logmap must be a Matrix.','Input Error');
    return
end

% Check Matrix Dimensions
[A,B] = size(exp_wv);
if A ~= B || A ~= 3 || B ~= 3
    errordlg('Input to se3_wedge must be an 3x3 Matrix.','Input Error');
    return
end

% Calculate angle of Rotation (theta)
% Trace of Rotation (SO(3)
trR = trace(exp_wv);
cos_theta = max(min(0.5*(trR - 1),1),-1);

% Use inverse tan to wrap theta
sin_theta = 0.5*sqrt(max(0,(3 - trR)*(1 + trR)));
theta = atan2(sin_theta,cos_theta);

%% Condition for approaching theta = pi
S = exp_wv + exp_wv' + (1 - trR)*eye(3);
n = ones(1,3);

for i = 1:3
    n(1,i) = max(0,S(i,i)/(3 - trR));
end
% Index of max value
[~,max_idx] = max(n);

% Correct signs
for i = 1:3
    if i~=max_idx
        n(1,i) = n(1,i)*sign(S(max_idx,i));
    end
end

% Correct overall sign
% Fix overall signs
nt = n';
if any(sign(n)*sign(so3_invwedge(exp_wv - exp_wv')) < 0)
    nt = -nt;
end


% Condition for theta ~0, use Taylor Series Expansion
if abs(theta) < 1e-9
    so3 = so3_invwedge(exp_wv - eye(length(SO3)));
% Take the Logarithm
else
    so3 = (theta/(2*sin(theta)))*so3_invwedge(exp_wv - exp_wv');
    % ln_exp_wv = (theta/(2*sin(theta)))*(exp_wv - exp_wv');
    % so3 = so3_invwedge(ln_exp_wv);
end

end