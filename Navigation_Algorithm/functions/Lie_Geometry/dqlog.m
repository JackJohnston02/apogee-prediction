function dql = dqlog(dq)
%  DQLOG Calculates the logarithm of a unit dual quaternion.
%   DQL = DQLOG(DQ) calculates the logarithm dual quaternion DQL of unit
%   dual quaternion DQ.
%
%   Input:
%   DQ:          1-by-8 array containing unit dual quaternion
%
%   Output:
%   DQL:         1-by-8 array containing pure dual quaternion
%
%   Example:
%
%   Calculate the logarithm of dual quaternion dq = [0 1 0 0 0 0 2 1]:
%
%   dql = dqlog(dq)
%
%   See also DQEXP, DQMULTIPLY, DQCONJ, DQINV, DQMULTIPLY, DQ2SCREW
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check input
validateattributes(dq,{'numeric'},{'ncols',8,'nrows',1})
if ~dqisunit(dq)
    error('Non-unit dual quaternion')
end
% Find screw parameters for dual quaternion
[ang,p,ax,m] = dq2screw(dq);
% Check if angle is zero and construct dual vector
if ang == 0
    dqv = [0,ax,0,0,0,0];
else
    dqv = [0,ax,0,m];
end
% Construct dual angle
dqang = 1/2*[ang,0,0,0,p,0,0,0];
% Dual quaternion logarithm is multiplication of dual angle and dual vector
dql = dqmultiply(dqv,dqang);
end