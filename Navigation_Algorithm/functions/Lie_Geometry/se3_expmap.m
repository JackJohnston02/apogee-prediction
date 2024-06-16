function [SE3] = se3_expmap(se3)
% Function tranforms from a se(3) Lie Algebra to a SE(3) Lie Group
% using the Exponential Map.
%
% Inputs:
% se3 = 3x1 Lie Algebra
%
% Outputs:
% SE3 = 3x3 SE(3) Lie Group Rotation Matrix
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

% Define Generator Matrices for SE(3)
G1 = zeros(4,4);
G2 = G1; G3 = G1; G3 = G1; G4 = G1; G5 = G1; G6 = G1;
G1(1,4) = 1;
G2(2,4) = 1;
G3(3,4) = 1;
G4(3,2) = 1;
G4(2,3) = -1;
G5(1,3) = 1;
G5(3,1) = -1;
G6(2,1) = 1;
G6(1,2) = -1;


% Transform the input if required
if isrow(se3)
    se3 = se3';
end

if length(se3) == 6
    % Split Lie Algebra into Components (u,w)
    % Translational Component
    t = se3(1:3);
    % Rotational Component
    w = se3(4:6);
else
    errordlg('Input to se3_expmap has incorrect dimensions.','Dimension Error');
    return
end

% Wedge Operation (w^) of (w) maps (w) to it's skew symmetric matrix
wv = se3_wedge(w);

% Angle of Rotation (theta)
theta = sqrt(w'*w);

% Condition for Zero Rotation
if theta == 0
    exp_wv = eye(3);
    V = eye(3);
else
    exp_wv = eye(3) + (sin(theta)/theta)*wv + ((1-cos(theta))/(theta^2))*wv^2;
    V = eye(3) + ((1-cos(theta))/(theta^2))*wv + ((theta-sin(theta))/(theta^3))*wv^2;
end

Vt = V*t;

% Generate Exponential Map
SE3 = zeros(4,4);
SE3(1:3,1:3) = exp_wv;
SE3(1:3,4) = Vt;
SE3(4,4) = 1;

end