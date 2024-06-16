function dq=tform2dq(tf)
%  TFORM2DQ Calculates unit dual quaternion from homogeneous transformation
%  matrix.
%   DQ = TFORM2DQ(TF) calculates the corresponding unit dual quaternion DQ
%   for homogeneous transformation matrix TF.
%
%   Inputs:
%   TF:         4-by-4 array containing homogeneous transformation matrix
%
%   Outputs:
%   DQ:         1-by-8 array containing unit dual quaternion
%
%   Example:
%   
%   Calculate corresponding unit dual quaternion for homogeneous
%   transformation matrix tf:
%
%       dq = tform2dq(tf)
%
%   See also DQ2TFORM, DQ2SCREW, DQ2TW
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check inputs

% Extract rotation matrix and translation vector
R = tf(1:3,1:3);
t = tf(1:3,4)';
% Rotation quaternion from rotation matrix
r = rotm2quat(R);
% Create unit dual quaternion
dq = dqdef(r,[0 t]);
end