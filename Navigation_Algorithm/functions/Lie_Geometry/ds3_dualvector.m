function [omega] = ds3_dualvector(w,v)
% Function to combine rotational and translational velocities into a dual
% vector for propagating a dual quaternion.
%
% Inputs:
% w - [3x1] Angular Velocity
% v - [3x1] Linear Velocity
%
% Outputs:
% omega - [8x1] Pure Dual Quaternion 
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
% Convert to column vectors
if width(w) > 1
    w = w';
end
if width(v) > 1
    v = v';
end

% Combine rotational and translational components
omega = [0; w; 0; v];

end