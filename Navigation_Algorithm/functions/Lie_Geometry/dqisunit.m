function q = dqisunit(dq)
%  DQISUNIT checks whether a dual quaternion is unit.
%   Q = DQISUNIT(DQ) checks whether the dual quaternion DQ is unit, i.e.
%   has norm equal to 1.
%
%   Input:
%   DQ:         1-by-8 array containing dual quaternion
%
%   Output:
%   Q:          Boolean indicating whether dual quaternion is unit
%
%   Example:
%
%   Check if the dual quaternion dq = [0 1 0 0 1 0 4 5] is unit:
%
%   q = dqisunit(dq)
%
%   See also DQISPURE
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check inputs
validateattributes(dq,{'numeric'},{'nrows',1,'ncols',8})
% Check if norm of dual quaternion is equal to 1
q = max(abs(dqnorm(dq)-[1 0 0 0 0 0 0 0])) < 1e-3;
end