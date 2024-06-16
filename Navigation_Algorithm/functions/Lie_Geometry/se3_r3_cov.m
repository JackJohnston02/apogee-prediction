function [P] = se3_r3_cov(X,Xh,z,zh)
% Function to calculate the NxN Covariance matrix of the SE(3)xR3 joint
% state. Performs the Box Minus operation before calculating the
% expectation related to the Mean State.
%
% Inputs:
% X - Mean SE(3) State
% Xh - Estimated SE(3) State
% z - Mean R3 Vector State
% zh - Estimated R3 Vector State
%
% Outputs:
% P - Covariance of the State
%
% Date Created: 19/05/2020
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
if ~isequal(size(X),[4,4]) || ~isequal(size(Xh),[4,4])
    errordlg('Input to se3_cov has incorrect dimensions.','Dimension Error');
    return;
end

%% Calculate Expectation P = E[(X (-) Xh)(X (-) Xh)']

% SE(3) Box Minus Operation
tau = se3_Ominus(X,Xh);

% R3 Vector State Linear Error
epsilon = z - zh;

% Combined Error State [t; e]
errorState = [tau; epsilon];

% Calculate Covariance
P = (errorState)*(errorState)';


%% END OF FUNCTION
end