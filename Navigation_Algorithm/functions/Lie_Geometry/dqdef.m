function dq = dqdef(qr,qt)
%  DQDEF Define a unit dual quaternion via rotation and translation
%   DQ = DQDEF(QR,QT) defines the unit dual quaternion DQ from the rotation
%   unit quaternion QR and translation pure quaternion QT.
%
%   Inputs:
%   QR:         1-by-4 unit quaternion representing rotation
%   QT:         1-by-4 pure quaternion representing translation
%
%   Output:
%   DQ:         1-by-8 unit dual quaternion
%
%   Example:
%
%   Define a unit dual quaternion with qr = [cos(pi/4) 0 sin(pi/4) 0] and
%   qt = [0 3 0 2]:
%
%   dq = dqdef(qr,qt)
%
%   See also SCREW2DQ, DQRAND
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check inputs
validateattributes(qr,{'numeric'},{'nrows',1,'ncols',4});
validateattributes(qt,{'numeric'},{'nrows',1,'ncols',4});
validateattributes(qt(1),{'numeric'},{'<=',eps,'>=',-eps});
validateattributes(qr(1)^2+qr(2)^2+qr(3)^2+qr(4)^2,{'numeric'},{'<=',1+1e-3,'>=',1-1e-3});
% Create dual quaternion
dq = [qr 1/2*quatmultiply(qt,qr)];
end