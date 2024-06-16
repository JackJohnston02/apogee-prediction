function n = dqnorm(dq)
%  DQNORM Calculates norm of a dual quaternion.
%   N = DQNORM(DQ) calculates the norm of dual quaternion DQ.
%
%   Input:
%   DQ:         1-by-8 array containing dual quaternion
%
%   Output:
%   N:          Scalar norm of dual quaternion
%
%   Example:
%   
%   Calculate norm of dual quaternion dq = [2 0 1 0 0 -1 0 -1]:
%
%   n = dqnorm(dq)
%
%   See also DQCONJ, DQISUNIT, DQMULTIPLY, DQINV
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check input
validateattributes(dq,{'numeric'},{'ncols',8,'nrows',1})
% Calculate dq.dq*
normsquare=dqmultiply(dq,dqconj(dq));
% Norm is dual square root of dq.dq*
n = [sqrt(normsquare(1)) 0 0 0 normsquare(5)/(2*sqrt(normsquare(1))) 0 0 0];
end