function tf=dq2tform(dq)
%  DQ2TFORM Calculates homogeneous transform matrix from unit dual
%  quaternion.
%   TF = DQ2TFORM(DQ) calculates the corresponding homogeneous
%   transformation matric TF for unit dual quaternion DQ.
%
%   Inputs:
%   DQ:
%
%   Outputs:
%   TF:
%
%   Example:
%   
%   Calculate corresponding homogeneous transformation for a random unit
%   dual quaternion:
%
%       tf = dq2tform(dqrand)
%
%   See also TFORM2DQ, DQ2SCREW, DQ2TWISTOR
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
%% Checks
validateattributes(dq,{'numeric'},{'nrows',1,'ncols',8});
if ~dqisunit(dq)
    error('Non-unit dual quaternion')
end
%% Method
% Real and dual parts
r = dq(1:4);
d = dq(5:8);
% Rotation matrix from rotation quaternion (real part)
R = quat2rotm(r);
% Translation vector (t = 2.d.r*)
t = 2*quatmultiply(d,quatconj(r));
% Construct homogeneous transformation matrix
tf = [R,t(2:4)';0 0 0 1];
end