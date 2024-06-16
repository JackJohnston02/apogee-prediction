function [DQc] = ds3_conjugate(DQ,varargin)
% Function to calculate the inverse of a dual quaternion.
%
% Inputs:
% DQ - [8x1] Pure Dual Quaternion
%
% Outputs:
% DQinv - [8x1] Inverse of Unit Dual Quaternion Product
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

% Check for formatting
if height(DQ) > 1
    DQ = DQ';
end

% Standard dual quaternion conjugate
DQc = [quatconj(DQ(:,1:4)),quatconj(DQ(:,5:8))];

% If conjugate type specified
if nargin > 1
    mode = varargin{1};
    if strcmp(mode,'classic')
        % If 'classic' conjugate specified, use standard conjugate above
        return
    elseif strcmp(mode,'dual')
        % If 'dual' conjugate specified, only negate dual part
        DQc = [DQ(:,1:4),-DQ(:,5:8)]; 
        return;
    elseif strcmp(mode,'both')
        % If 'both' conjugate specified, quaternion counjugate and negate dual part
        DQc = [quatconj(DQ(:,1:4)),-quatconj(DQ(:,5:8))]; 
        return;
    else
        error('Conjugate not recognised! Must be ''classic'', ''dual'' or ''both''')
    end
end

end