function [DQp] = ds3_product(varargin)
% Function to perform dual quaternion multiplication.
%
% Inputs:
% DQ - [8x1] Pure Dual Quaternion Lie Algebra
%
% Outputs:
% DQp - [8x1] Unit Dual Quaternion Product
%
% Date Created: 01/02/2022
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
% Quaternion Kinematics of the Error-State Kalman Filter - Sola [2012]
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]
%
% http://jamessjackson.com/lie_algebra_tutorial/06-closed_form_mat_exp/
%
% https://uk.mathworks.com/matlabcentral/fileexchange/71397-robotics-dual-quaternion-toolbox

% Quaternion Multiplication requires row vectors

% Get number of products
switch nargin
    case {0,1}
        error('Require two or more dual quaternions.');
    case 2
        dq1 = varargin{1};
        dq2 = varargin{2};
        % Convert to Row Format if required
        if height(dq1) > 1
            dq1 = dq1';
        end
        if height(dq2) > 1
            dq2 = dq2';
        end
        % Perform Dual Quaternion Product
        DQp = [quatmultiply(dq1(1:4),dq2(1:4))...
               quatmultiply(dq1(1:4),dq2(5:8))+quatmultiply(dq1(5:8),dq2(1:4))];
    otherwise
        % Perform successive products
        DQp = ds3_product(varargin{1},ds3_product(varargin{2:end}));
end

% Convert to column quaternion
DQp = DQp';

end


