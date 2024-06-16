function dqx=dqexp(dq)
%  DQEYE Calculates the exponential of a pure dual quaternion.
%   DQX = DQEXP(DQ) calculates the exponential of the pure dual quaternion
%   DQ and returns the unit dual quaternion DQX.
%
%   Inputs:
%   DQ:         1-by-8 array containing pure dual quaternion
%
%   Outputs:
%   DQX:        1-by-8 array containing unit dual quaternion
%
%   Example:
%
%   Calculate the exponential of dual quaternion dq = [0 1 0 2 0 -1 -1 3]:
%
%       dqx = dqexp(dq)
%
%   See also DQLOG, DQMULTIPLY, DQPOWER
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check inputs
w0 = dq(1); w = dq(2:4); v0 = dq(5); v = dq(6:8);
if abs(w0) > 1e-3 || abs(v0) > 1e-3
    error('Non-pure dual quaternion')
end
% Norm of rotation component
w_norm = norm(w);
% If no rotation, return trivial resultant unit dual quaternion
if abs(w_norm) < eps
    dqx=[1 0 0 0 0 v];
    return
end
% Axis of rotation
w_unit = w/w_norm;
% Pitch
d = 2*w_unit*v';
% Calculate exponential according to [1]
m = (v - 1/2*d*w_unit)/w_norm;
a = 1/2*d*cos(w_norm);
dqx = [cos(w_norm), sin(w_norm)*w_unit,... 
       -1/2*d*sin(w_norm), sin(w_norm)*m + a*w_unit];
end
% [1] blah