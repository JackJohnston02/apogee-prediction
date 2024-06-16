function [Xh] = s3_R3_leftOminus(X,Xe)
% Function to perform the Box Minus (-) operation on elements M2, M1 of
% the S3 and SO3 Special Orthogonal Groups.
%
% Inputs:
% M2,M1 - S3/SO(3) Manifolds / S3/SO(3) States
%
% Outputs:
% wv - 3x1 Box Minus State in a Rotation Vector representation 
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
% Nonparametric Second-order Theory of Error Propagation on Motion
% Groups - Wang, Chrikijian [2008]
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

% Check Length of Element X
[n,~] = size(X);

switch n
    % Element X is a Rotation Vector in se(3)
    case 6
        Xh = X - Xe;
    % Element X is a Quaternion in S3
    case 7
        Xeq = s3_expmap(Xe(1:3));
        Xh(1:3,1) = s3_logmap(real(quatmultiply(X(1:4)',quatinv(Xeq'))'));
        Xh(4:6,1) = X(5:7) - Xe(4:6);
    otherwise
        Xh = X;
end



end
