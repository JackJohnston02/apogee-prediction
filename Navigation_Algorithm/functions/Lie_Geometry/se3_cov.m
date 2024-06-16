function [P] = se3_cov(X,Xh)
% Function to calculate the NxN Covariance matrix of the SE(3) Matrix
% state. Performs the Box Minus operation before calculating the
% expectation related to the Mean State.
%
% Inputs:
% X - Mean SE(3) State
% Xh - Estimated SE(3) State
%
% Outputs:
% P - Covariance of the State
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
if ~isequal(size(X),[4,4]) || ~isequal(size(Xh),[4,4])
    errordlg('Input to se3_cov has incorrect dimensions.','Dimension Error');
    return;
end

%% Calculate Expectation P = E[(X (-) Xh)(X (-) Xh)']

% SE(3) Box Minus Operation
tau = se3_Ominus(X,Xh);

% Calculate Covariance
P = tau*tau';


%% END OF FUNCTION
end