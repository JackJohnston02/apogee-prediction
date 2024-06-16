function [Jlinv_se3] = se3_leftInvJacobian(se3)
% Function to calculate the Left Inverse Jacobian Matrix of a SE(3) 
% Lie Algebra or Lie Group.
%
% Inputs:
% se3 = 3x1 Lie Algebra
%
% Outputs:
% Jrinv_se3 = 3x3 Right Inverse Jacobian Matrix of SE(3)
%
% Date Created: 12/11/2020
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
%
% Vision Based Navigation Course Notes - Usenko, Demmel, Schubert, 
% Cremers [2020]

% Transform the input if required
if isrow(se3)
    se3 = se3';
end

if length(se3) == 3
    % Define Lie Algebra as Rotation Vector
    w = se3;
elseif length(se3) == 6
    % Select Rotational Component
    w = se3(4:6);
else
    errordlg('Input to se3_leftJacobian has incorrect dimensions.','Dimension Error');
    return
end

% Calculate angle of Rotation (theta)
theta = norm(w);

% Calulate Wedge Operation
wv = se3_wedge(w);

% Condition for theta~0, use a First Order Taylor Series Expansion
if abs(theta) < 1e-9
    Jlinv_se3 = eye(3) - wv/2;
else
    Jlinv_se3 = eye(3) - 0.5*wv + (1/(theta^2) - ...
        (1 + cos(theta))/(2*theta*sin(theta)))*wv^2;
end

end