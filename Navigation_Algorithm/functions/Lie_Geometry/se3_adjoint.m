function [adjSE3] = se3_adjoint(SE3)
% Function to compute the Adjoint of SE(3).
%
% Inputs:
% SE3 = 3x3 Rotation Matrix in the Group SE(3)
%
% Outputs:
% adjSE3 = 3x3 Matrix
%
% Date Created: 11/11/2020
%
% Created by: Joe Gibbs
%
% References:
%
% Lie Groups for 2D and 3D Transformations - Eade [2017]

% Check Matrix Input
if ~ismatrix(SE3)
    errordlg('Input to se3_adjoint must be a Matrix.','Input Error');
    return
end

% Check Matrix Dimensions
[A,B] = size(SE3);
if A ~= B || A ~= 4 || B ~= 4
    errordlg('Input to se3_adjoint must be an 4x4 Matrix.','Input Error');
    return
end


% Define Rotational and Transaltional Elements
R = SE3(1:3,1:3);
t = [SE3(1:3,4)];

% Wedge Operation for Translational Component
tv = se3_wedge(t);

% Calculate Adjoint
adjSE3 = [R tv*R; zeros(3) R];

end