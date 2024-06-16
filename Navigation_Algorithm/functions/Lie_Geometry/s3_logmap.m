function [wv] = s3_logmap(q)
% Function to tranform from a S3 Lie Group to a SO(3) Lie Algebra
% using the Logarithmic Map of S3.
%
% Inputs:
% q - S3 Manifold ELement
%
% Outputs:
% wv - Rotation Vector Element of se(3)
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

% Remove Imaginary Quaternion Elements
q = real(q);

% % Angle and Direction
% if q(1) >= 0
%     theta = 2*acos(q(1));
% else
%     theta = -2*acos(-q(1));
% end
% 
% % Calculate Tangent Space Element
% if norm(q(2:4)) > 1e-10
%     wv = theta*q(2:4)/norm(q(2:4));
% else
%     wv = 2*q(2:4);
% end
% 
% % Ensure Real Elements
% wv = real(wv);

%% Numerically stable method
q = sign(q(1))*q;

qw = q(1);
qv = q(2:4);

% Calculate Tangent Space Element
if norm(q(2:4)) > 1e-10
    wv = 4*atan((norm(qv)^2)/(qw + sqrt(qw^2 + norm(qv)^2)))*(qv/norm(qv)^2);
else
    wv = (2/qw-2/3*norm(qv)^2/qw^3)*qv;
end

% Ensure Real Elements
wv = real(wv);

end

