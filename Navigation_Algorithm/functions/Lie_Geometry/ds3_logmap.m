function [DQT] = ds3_logmap(DQ)
% Function to tranform from a DS^3 Lie Group to a DS^3 Lie Algebra
% using the Logarithmic Map of the dual quaternion Lie Group.
%
% Inputs:
% DQX - [8x1] Unit Dual Quaternion
%
% Outputs:
% DQ - [8x1] Pure Dual Quaternion Lie Algebra
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

%% Calculate the Screw Motion Parameters corresponding to the Dual Quaternion
% Real
r = DQ(1:4)';
% Dual
d = DQ(5:8)';

% Translation Component [0; t];
t = 2*quatmultiply(d,quatconj(r));
t = t(2:4)';

% Angle-Axis Components
axis_angle = quat2axang(r);
angle = axis_angle(4);
axis = axis_angle(1:3)';

% Condition for pure translation (no rotation)
if abs(angle) < eps && norm(t) ~= 0
    axis = t/norm(t)';
end

% Pitch
p = dot(t,axis);

% Moment
m = 0.5*(cross(t,axis) + (t - p*axis)*cot(angle/2));

%% Assemble the Dual Quaternion
if angle == 0
    DQv = [0; axis; zeros(4,1)];
else
    DQv = [0; axis; 0; m];
end

% Construct the Dual Angle
phi = 0.5*[angle; zeros(3,1); p; zeros(3,1)]';

%% Dual Quaternion Logarithm
% Logarithm is the product of Dual Angle and Dual Vector
DQT = [quatmultiply(DQv(1:4)',phi(1:4))';...
       quatmultiply(DQv(1:4)',phi(5:8))' + quatmultiply(DQv(5:8)',phi(1:4))'];


end


