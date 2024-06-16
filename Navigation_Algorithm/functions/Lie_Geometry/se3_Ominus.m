function [tau] = se3_Ominus(M2,M1)
% Function to perform the Box Minus (-) operation on two elements M2, M1 of
% the SE(3) Special Euclidean Group.
%
% Inputs:
% M2,M1 - SE(3) Manifolds / SE(3) States
%
% Outputs:
% tau - Box Minus State of the form [t; omega]
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
% A Micro Lie Theory for State Estimation in Robotics - Sola, Deray,
% Atchuthan [2020]
%
% Nonparametric Second-order Theory of Error Propagation on Motion
% Groups - Wang, Chrikijian [2008]
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

%% Input Checks
% Group Dimensions
if ~isequal(size(M1),[4,4]) || ~isequal(size(M2),[4,4])
    errordlg('Input to se3_Ominus has incorrect dimensions.','Dimension Error');
    return;
end


%% Separate Elements
R1 = M1(1:3,1:3);
p1 = M1(1:3,4);

R2 = M2(1:3,1:3);
p2 = M2(1:3,4);


%% Perform Box Minus Operation
% SE(3) Log Map to transform State to the Tangent space se(3)
m1 = se3_logmap(M1);

% Check Length and Split Element
if length(m1) == 6
    % Split Lie Algebra into Components (u,w)
    % Rotational Component
    w = m1(4:6);
else
    errordlg('Input to se3_expmap has incorrect dimensions.','Dimension Error');
    return
end

% Wedge Operation (w^) of (w) maps (w) to it's skew symmetric matrix
wv = so3_wedge(w);

% Angle of Rotation (theta)
theta = norm(w);

% Condition for Zero Rotation
if theta == 0
    V1 = eye(3);
else
    V1 = eye(3) + ((1-cos(theta))/(theta^2))*wv + ((theta-sin(theta))/(theta^3))*wv^2;
end

% Box Minus Operation

tau = [(V1^-1)*R1'*(p2 - p1);...
      so3_logmap(R1'*R2)];


%% END OF FUNCTION
end