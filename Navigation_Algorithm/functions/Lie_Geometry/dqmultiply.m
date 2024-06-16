function dqm = dqmultiply(varargin)
%  DQMULTIPLY Calculates the multiplication of dual quaternions.
%   DQM = DQNORM(DQ1, DQ2, DQ3) calculates the multiplication of dual
%   quaternions DQ1, DQ2 and DQ3 and returns DQM.
%
%   Inputs:
%   DQ1,...,DQN:    1-by-8 arrays containing dual quaternions
%
%   Output:
%   DQM:            1-by-8 array containing resultant dual quaternion
%
%   Example:
%
%   Calculate the multiplication of 4 random unit dual quaternions:
%
%   dqm = dqmultiply(dqrand,dqrand,dqrand,dqrand);
%   dqdisp(dqm)
%
%   See also DQCONJ, DQINV, DQEXP, DQLOG, DQNORM, DQRAND
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check inputs
for ii=1:nargin
    validateattributes(varargin{ii},{'numeric'},{'ncols',8,'nrows',1})
end
switch nargin
    case {0,1}
        error('Require at least 2 dual quaternions to multiply');
    case 2
        % Multiply 2 dual quaternions
        dq1 = varargin{1};
        dq2 = varargin{2};
        dqm = [quatmultiply(dq1(1:4),dq2(1:4)),...
               quatmultiply(dq1(1:4),dq2(5:8)) + ...
               quatmultiply(dq1(5:8),dq2(1:4))];
    otherwise
        % Use recursion to multiply > 2 dual quaternions
        dqm=dqmultiply(varargin{1},dqmultiply(varargin{2:end}));
end