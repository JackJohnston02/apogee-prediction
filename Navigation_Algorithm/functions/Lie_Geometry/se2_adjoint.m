function [adjSE2] = se2_adjoint(SE2)
% Function to compute the Adjoint of SE(3).
%
% Inputs:
% SE2 = 3x3 Matrix in the Group SE(3)
%
% Outputs:
% adjSE2 = 3x3 Matrix
%
% Date Created: 11/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

% Check Matrix Input
if ~ismatrix(SE2)
    errordlg('Input to se2_adjoint must be a Matrix.','Input Error');
    return
end

% Check Matrix Dimensions
[A,B] = size(SE2);
if A ~= B || A ~= 3 || B ~= 3
    errordlg('Input to se2_adjoint must be an 3x3 Matrix.','Input Error');
    return
end


% Define Rotational and Transaltional Elements
R = SE2(1:2,1:2);
t = [SE2(2,3); -SE2(1,3)];

% Calculate Adjoint
adjSE2 = [t R; 1 0 0];

end