function [q] = s3_expmap(wv)
% Function to tranform from a SO(3) Lie Algebra to a S3 Lie Group
% using the Exponential Map of S3.
%
% Inputs:
% wv - Rotation Vector Element of se(3)
%
% Outputs:
% q - S3 Manifold Element
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
%   To remove the ambiguity about the direction, the argument of arccos is
%   checked for non-negativity.

% Remove Imaginary Rotation Vector Elements
wv = real(wv);

% Calculate Rotation Angle
theta = norm(wv);

% Calculate Quaternion Element on S3
if theta > 1e-10
    q(1,1) = cos(theta/2);
    q(2:4,1) = sin(theta/2)*wv/theta;
else
    q(1,1) = cos(theta/2);
    q(2:4,1) = sin(theta/2)*ones(3,1);
end

% Ensure Real Elements
q = real(q);

end


