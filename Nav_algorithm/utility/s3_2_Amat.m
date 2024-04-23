function [Aq] = s3_2_Amat(q)
% Function to calculate the equivalent Attitude Matrix related to an 
% attitude quaternion.
%
% Inputs:
% q = 4x1 Unit Quaternion
%
% Outputs:
% A(q) = 3x3 matrix
%
% Date Created: 23/11/2022
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
% Quaternion Kinematics for the Error-State Kalman Filter - Sola [2012]
% Table 1, pg 15

qw = q(1);
qv = q(2:4);

% Sola
% Aq = (qw^2 - qv'*qv)*eye(3) + 2*qv*qv' + 2*qw*so3_wedge(qv);
% Gui
Aq = (qw^2 - (qv')*qv)*eye(3) + 2*qv*(qv') - 2*qw*so3_wedge(qv);

end
