function dqc=dqconj(dq,varargin)
%  DQCONJ Calculate the conjugate of a dual quaternion.
%   DQC = DQCONJ(DQ,TYPE) calculates the conjugate, N, for a given dual
%   quaternion DQ, with the optional parameter TYPE.
%   
%   Inputs:
%   DQ:             M-by-8 matrix containing M dual quaternions
%   TYPE:           String specifying the type of conjugate desired. The
%                   options for TYPE are:
%                   'classic' - (Default) Standard quaternion conjugate
%                   'dual'    - Dual part conjugate
%                   'both'    - Both quaternion and dual conjugate
%
%   Output:
%   DQC:            M-by-8 matrix containing M dual quaternions.
%
%   Examples:
%
%   Determine the conjugate of dq = [1 0 1 0 0 1 3 0]:
%
%       dqc = dqconj([1 0 1 0 0 1 3 0])
%
%   Determine the dual conjugate of dq = [1 0 1 0 0 1 3 0]:
%
%       dqc = dqconj([1 0 1 0 0 1 3 0],'dual')
%
%   See also DQNORM, DQINV, DQMULTIPLY.
%   Author:     Thomas Watts
%   Work:       Imperial College London
%   Email:      t.watts14@imperial.ac.uk
%   Date:       31/7/2018
% Check inputs
narginchk(1, 2);
validateattributes(dq,{'numeric'},{'ncols',8});
% Standard dual quaternion conjugate
dqc=[quatconj(dq(:,1:4)),quatconj(dq(:,5:8))];
% If conjugate type specified
if nargin>1
    mode=varargin{1};
    if strcmp(mode,'classic')
        % If 'classic' conjugate specified, use standard conjugate above
        return
    elseif strcmp(mode,'dual')
        % If 'dual' conjugate specified, only negate dual part
        dqc=[dq(:,1:4),-dq(:,5:8)]; return
    elseif strcmp(mode,'both')
        % If 'both' conjugate specified, quaternion counjugate and negate dual part
        dqc=[quatconj(dq(:,1:4)),-quatconj(dq(:,5:8))]; return
    else
        error('Conjugate not recognised! Must be ''classic'', ''dual'' or ''both''')
    end
end